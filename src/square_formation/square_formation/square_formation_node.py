import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from example_interfaces.msg import Empty
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import math

class SquareFormationNode(Node):
    def __init__(self):
        super().__init__('square_formation_node')

        # Shared robot list across all instances
        self.robots = ['robot1', 'robot2', 'robot3', 'robot4']

        # Parameters
        self.declare_parameter('robot_name', 'robot1')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        # Square positions (edit as needed)
        self.positions = {
            'robot1': (0.0, 0.0),
            'robot2': (1.0, 0.0),
            'robot3': (1.0, 1.0),
            'robot4': (0.0, 1.0),
        }

        # State
        self.target_pose = self.positions[self.robot_name]
        self.current_pose = (0.0, 0.0)
        self.current_yaw = 0.0
        self.has_goal = False           # set True on /left or /right
        self.started_moving = False     # becomes True once all aligned

        # Alignment status of all robots (including self)
        self.alignment_statuses = {r: False for r in self.robots}

        # QoS for alignment topics: Transient Local + Reliable (latch-like)
        aligned_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # IO
        self.create_subscription(Odometry, f'/{self.robot_name}/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', 10)

        self.create_subscription(Empty, '/left', self.left_callback, 10)
        self.create_subscription(Empty, '/right', self.right_callback, 10)

        # Subscribe to all robots' /aligned with matching QoS
        for robot in self.robots:
            self.create_subscription(Bool, f'/{robot}/aligned', self.aligned_callback_factory(robot), aligned_qos)

        # Publish this robot's alignment with same QoS
        self.aligned_pub = self.create_publisher(Bool, f'/{self.robot_name}/aligned', aligned_qos)

        # Control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Tunables for approach and finalization
        self.angle_threshold = 0.12     # rad; require decent alignment before starting
        self.dist_threshold = 0.10      # m; general stopping radius (kept for safety)
        self.stop_dist = 0.06           # m; final acceptance radius (tighter)
        self.final_yaw_thresh = 0.15    # rad; yaw tolerance at goal
        self.stop_dwell_cycles = 8      # consecutive cycles inside tolerances to declare reached
        self.in_goal_count = 0          # dwell counter

        # Motion gains and limits
        self.max_angular_speed = 0.6    # rad/s
        self.v_max = 0.18               # m/s
        self.k_ang = 2.0                # proportional heading gain
        self.k_lin = 0.8                # proportional distance gain
        self.slowdown_dist = 0.40       # m; begin aggressive slowing
        self.v_near_max = 0.08          # m/s; cap when near
        self.v_min = 0.03               # m/s; minimum forward when commanded

    def odom_callback(self, msg):
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def left_callback(self, _):
        idx = self.robots.index(self.robot_name)
        neighbor = self.robots[(idx - 1) % len(self.robots)]
        self.target_pose = self.positions[neighbor]
        self.has_goal = True
        self.started_moving = False
        self.alignment_statuses[self.robot_name] = False
        self.publish_aligned(False)
        self.in_goal_count = 0
        self.get_logger().info(f'{self.robot_name}: LEFT -> {neighbor} at {self.target_pose}')

    def right_callback(self, _):
        idx = self.robots.index(self.robot_name)
        neighbor = self.robots[(idx + 1) % len(self.robots)]
        self.target_pose = self.positions[neighbor]
        self.has_goal = True
        self.started_moving = False
        self.alignment_statuses[self.robot_name] = False
        self.publish_aligned(False)
        self.in_goal_count = 0
        self.get_logger().info(f'{self.robot_name}: RIGHT -> {neighbor} at {self.target_pose}')

    def aligned_callback_factory(self, robot):
        def cb(msg: Bool):
            self.alignment_statuses[robot] = bool(msg.data)
        return cb

    def control_loop(self):
        twist = Twist()

        if not self.has_goal:
            self.publish_aligned(False)
            self.cmd_vel_pub.publish(twist)
            return

        # Geometry to target
        x, y = self.current_pose
        tx, ty = self.target_pose
        dx, dy = (tx - x), (ty - y)
        target_angle = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(target_angle - self.current_yaw)
        distance = math.hypot(dx, dy)

        # Phase 1: Align before starting
        if not self.started_moving:
            if abs(angle_diff) > self.angle_threshold:
                twist.linear.x = 0.0
                twist.angular.z = self.clamp(self.k_ang * angle_diff, -self.max_angular_speed, self.max_angular_speed)
                self.publish_aligned(False)
                self.cmd_vel_pub.publish(twist)
                return
            self.alignment_statuses[self.robot_name] = True
            self.publish_aligned(True)
            if self.all_aligned():
                self.started_moving = True
                self.in_goal_count = 0

        # Phase 2: Guided approach with continuous heading correction
        if self.started_moving:
            if distance > self.stop_dist:
                # Proportional heading correction while moving
                ang_cmd = self.clamp(self.k_ang * angle_diff, -self.max_angular_speed, self.max_angular_speed)

                # Distance-based speed, reduced when misaligned and near goal
                v_cmd = min(self.v_max, self.k_lin * distance)
                v_cmd *= max(0.0, math.cos(angle_diff))
                if distance < self.slowdown_dist:
                    v_cmd = min(v_cmd, self.v_near_max)
                if v_cmd > 0.0:
                    v_cmd = max(v_cmd, self.v_min)

                twist.linear.x = v_cmd
                twist.angular.z = ang_cmd
                self.in_goal_count = 0
            else:
                # Finalization window: stop linear motion and optionally trim yaw, then dwell
                twist.linear.x = 0.0
                if abs(angle_diff) > self.final_yaw_thresh:
                    twist.angular.z = self.clamp(self.k_ang * angle_diff, -self.max_angular_speed, self.max_angular_speed)
                else:
                    twist.angular.z = 0.0
                self.in_goal_count += 1
                if self.in_goal_count >= self.stop_dwell_cycles:
                    # Declare reached; ready for next /left or /right
                    self.has_goal = False
                    self.started_moving = False
                    self.publish_aligned(False)
                    self.in_goal_count = 0

        self.cmd_vel_pub.publish(twist)

    def all_aligned(self):
        return all(self.alignment_statuses.values())

    def publish_aligned(self, aligned: bool):
        self.aligned_pub.publish(Bool(data=aligned))

    @staticmethod
    def clamp(x, lo, hi):
        return max(lo, min(hi, x))

    @staticmethod
    def normalize_angle(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

def main():
    rclpy.init()
    node = SquareFormationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

