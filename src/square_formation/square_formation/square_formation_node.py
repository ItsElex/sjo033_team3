import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from example_interfaces.msg import Empty
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
import math

class SquareFormationNode(Node):
    def __init__(self):
        super().__init__('square_formation_node')

        self.robots = ['robot1','robot2','robot3','robot4']
        self.declare_parameter('robot_name','robot1')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        # Shared square corners defined in robot1/odom (global reference for formation)
        self.global_frame = 'robot1/odom'
        self.corners_global = [(0.0,0.0),(1.0,0.0),(1.0,1.0),(0.0,1.0)]

        # TF buffer/listener for inter-robot transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Local state (also keep local odom yaw for publishing cmd_vel stability if needed)
        self.current_corner_idx = 0
        self.target_corner_idx = 0
        self.target_global = self.corners_global[self.target_corner_idx]
        self.has_goal = False
        self.started_moving = False

        # Alignment flags
        self.alignment_statuses = {r: False for r in self.robots}

        aligned_qos = QoSProfile(depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        # IO
        self.create_subscription(Odometry, f'/{self.robot_name}/odom', self.odom_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', 10)
        self.create_subscription(Empty, '/left', self.left_cb, 10)
        self.create_subscription(Empty, '/right', self.right_cb, 10)
        for robot in self.robots:
            self.create_subscription(Bool, f'/{robot}/aligned', self._aligned_cb_factory(robot), aligned_qos)
        self.aligned_pub = self.create_publisher(Bool, f'/{self.robot_name}/aligned', aligned_qos)

        # Control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Tunables
        self.angle_threshold = 0.15
        self.stop_dist = 0.06
        self.final_yaw_thresh = 0.15
        self.stop_dwell_cycles = 8
        self.in_goal_count = 0

        self.max_angular_speed = 0.6
        self.v_max = 0.18
        self.k_ang = 2.5
        self.k_lin = 0.8
        self.slowdown_dist = 0.40
        self.v_near_max = 0.08
        self.v_min = 0.03

        # Track latest yaw from local odom (used only for publishing status/logging)
        self.current_yaw_local = 0.0

    def odom_cb(self, msg: Odometry):
        # Keep local yaw from odom in case you need it for monitoring
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw_local = math.atan2(siny_cosp, cosy_cosp)

    def left_cb(self, _):
        self._new_cmd(-1)

    def right_cb(self, _):
        self._new_cmd(+1)

    def _new_cmd(self, direction:int):
        # Determine current corner index from current pose transformed into robot1/odom
        pose_ok, px, py, yaw_g = self._self_in_global()
        if pose_ok:
            nearest_idx = self._nearest_corner_index((px,py), self.corners_global)
            self.current_corner_idx = nearest_idx
        # Pick next corner in global frame
        self.target_corner_idx = (self.current_corner_idx + direction) % len(self.corners_global)
        self.target_global = self.corners_global[self.target_corner_idx]

        # Reset barrier
        for k in self.alignment_statuses:
            self.alignment_statuses[k] = False
        self._publish_aligned(False)

        self.has_goal = True
        self.started_moving = False
        self.in_goal_count = 0
        self.get_logger().info(f'{self.robot_name}: cmd -> corner {self.target_corner_idx} at {self.target_global}')

    def _aligned_cb_factory(self, robot):
        def cb(msg: Bool):
            self.alignment_statuses[robot] = bool(msg.data)
        return cb

    def control_loop(self):
        tcmd = Twist()

        if not self.has_goal:
            self._publish_aligned(False)
            self.cmd_vel_pub.publish(tcmd)
            return

        # Get our current pose in the shared global frame robot1/odom
        ok, xg, yg, yaw_g = self._self_in_global()
        if not ok:
            # wait for TF
            self._publish_aligned(False)
            self.cmd_vel_pub.publish(Twist())
            return

        # Geometry in robot1/odom to global target
        tx, ty = self.target_global
        dx, dy = (tx - xg), (ty - yg)
        target_angle = math.atan2(dy, dx)
        angle_diff = self._norm_ang(target_angle - yaw_g)
        dist = math.hypot(dx, dy)

        # Phase 1: align before start
        if not self.started_moving:
            if abs(angle_diff) > self.angle_threshold:
                tcmd.linear.x = 0.0
                tcmd.angular.z = self._clamp(self.k_ang * angle_diff, -self.max_angular_speed, self.max_angular_speed)
                self._publish_aligned(False)
                self.cmd_vel_pub.publish(tcmd)
                return
            self.alignment_statuses[self.robot_name] = True
            self._publish_aligned(True)
            if self._all_aligned():
                self.started_moving = True
                self.in_goal_count = 0

        # Phase 2: move with continuous heading correction
        if self.started_moving:
            if dist > self.stop_dist:
                ang_cmd = self._clamp(self.k_ang * angle_diff, -self.max_angular_speed, self.max_angular_speed)
                v_cmd = min(self.v_max, self.k_lin * dist)
                v_cmd *= max(0.0, math.cos(angle_diff))
                if dist < self.slowdown_dist:
                    v_cmd = min(v_cmd, self.v_near_max)
                if v_cmd > 0.0:
                    v_cmd = max(v_cmd, self.v_min)
                tcmd.linear.x = v_cmd
                tcmd.angular.z = ang_cmd
                self.in_goal_count = 0
            else:
                tcmd.linear.x = 0.0
                if abs(angle_diff) > self.final_yaw_thresh:
                    tcmd.angular.z = self._clamp(self.k_ang * angle_diff, -self.max_angular_speed, self.max_angular_speed)
                else:
                    tcmd.angular.z = 0.0
                self.in_goal_count += 1
                if self.in_goal_count >= self.stop_dwell_cycles:
                    self.current_corner_idx = self.target_corner_idx
                    self.has_goal = False
                    self.started_moving = False
                    self._publish_aligned(False)
                    self.in_goal_count = 0

        self.cmd_vel_pub.publish(tcmd)

    def _self_in_global(self):
        # Returns (ok, x, y, yaw) of this robot in robot1/odom using TF
        try:
            now = rclpy.time.Time()
            target = self.global_frame               # 'robot1/odom'
            source = f'{self.robot_name}/base_link'  # this robot pose
            if not self.tf_buffer.can_transform(target, source, now, timeout=Duration(seconds=0.5)):
                return False, 0.0, 0.0, 0.0
            tf = self.tf_buffer.lookup_transform(target, source, now)
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            q = tf.transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            return True, x, y, yaw
        except Exception:
            return False, 0.0, 0.0, 0.0

    @staticmethod
    def _nearest_corner_index(pose_xy, corners):
        x, y = pose_xy
        best_i, best_d = 0, float('inf')
        for i, (cx, cy) in enumerate(corners):
            d = (cx - x)*(cx - x) + (cy - y)*(cy - y)
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    def _all_aligned(self):
        return all(self.alignment_statuses.values())

    def _publish_aligned(self, aligned: bool):
        self.aligned_pub.publish(Bool(data=aligned))

    @staticmethod
    def _clamp(x, lo, hi):
        return max(lo, min(hi, x))

    @staticmethod
    def _norm_ang(a):
        while a > math.pi:
            a -= 2*math.pi
        while a < -math.pi:
            a += 2*math.pi
        return a

def main():
    rclpy.init()
    n = SquareFormationNode()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

