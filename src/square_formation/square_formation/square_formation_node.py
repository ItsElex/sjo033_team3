import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from example_interfaces.msg import Empty
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import math
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs

class SquareFormationNode(Node):
    def __init__(self):
        super().__init__('square_formation_node')
        
        # TF2 setup for coordinate transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Robots for synchronized alignment
        self.robots = ['robot1', 'robot2', 'robot3', 'robot4']
        
        # Parameters
        self.declare_parameter('robot_name', 'robot1')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        
        # Square corners in order (CCW) in the GLOBAL map frame
        self.corners = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
        
        # State
        self.current_pose = (0.0, 0.0)
        self.current_yaw = 0.0
        self.current_corner_idx = 0
        self.target_corner_idx = 0
        self.target_pose = self.corners[self.target_corner_idx]
        self.has_goal = False
        self.started_moving = False
        
        # Alignment flags across robots (for synchronized start)
        self.alignment_statuses = {r: False for r in self.robots}
        
        # QoS for aligned (latched-like)
        aligned_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
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
        
        # Approach and goal-checking tunables
        self.angle_threshold = 0.2
        self.stop_dist = 0.08
        self.final_yaw_thresh = 0.2
        self.stop_dwell_cycles = 10
        self.in_goal_count = 0
        
        # Motion gains and limits
        self.max_angular_speed = 0.5
        self.v_max = 0.15
        self.k_ang = 1.8
        self.k_lin = 0.7
        self.slowdown_dist = 0.40
        self.v_near_max = 0.08
        self.v_min = 0.03
        
        # Snap tolerance for selecting current corner at move start
        self.snap_tol = 0.15
        
        self.get_logger().info(f'{self.robot_name}: Square formation node initialized')
    
    def odom_cb(self, msg: Odometry):
        try:
            pose_in_odom = PoseStamped()
            pose_in_odom.header = msg.header
            pose_in_odom.pose = msg.pose.pose
            
            pose_in_map = self.tf_buffer.transform(
                pose_in_odom,
                'map',
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            self.current_pose = (pose_in_map.pose.position.x, pose_in_map.pose.position.y)
            
            q = pose_in_map.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
            
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform: {ex}')
            return
    
    def left_cb(self, _):
        if self.has_goal or self.started_moving:
            return
        self._new_command(direction=-1)
    
    def right_cb(self, _):
        if self.has_goal or self.started_moving:
            return
        self._new_command(direction=+1)
    
    def _new_command(self, direction: int):
        nearest_idx = self._nearest_corner_index(self.current_pose)
        if self._dist2(self.current_pose, self.corners[nearest_idx]) <= self.snap_tol * self.snap_tol:
            self.current_corner_idx = nearest_idx
        
        self.target_corner_idx = (self.current_corner_idx + direction) % len(self.corners)
        self.target_pose = self.corners[self.target_corner_idx]
        
        for k in self.alignment_statuses:
            self.alignment_statuses[k] = False
        self._publish_aligned(False)
        
        self.has_goal = True
        self.started_moving = False
        self.in_goal_count = 0
        
        self.get_logger().info(f'{self.robot_name}: Moving to corner {self.target_corner_idx}')
    
    def _aligned_cb_factory(self, robot):
        def cb(msg: Bool):
            self.alignment_statuses[robot] = bool(msg.data)
        return cb
    
    def control_loop(self):
        twist = Twist()
        
        if not self.has_goal:
            self._publish_aligned(False)
            self.cmd_vel_pub.publish(twist)
            return
        
        x, y = self.current_pose
        tx, ty = self.target_pose
        dx, dy = (tx - x), (ty - y)
        target_angle = math.atan2(dy, dx)
        angle_diff = self._norm_ang(target_angle - self.current_yaw)
        distance = math.hypot(dx, dy)
        
        if not self.started_moving:
            if abs(angle_diff) > self.angle_threshold:
                twist.linear.x = 0.0
                twist.angular.z = self._clamp(self.k_ang * angle_diff, -self.max_angular_speed, self.max_angular_speed)
                self._publish_aligned(False)
                self.cmd_vel_pub.publish(twist)
                return
            
            self.alignment_statuses[self.robot_name] = True
            self._publish_aligned(True)
            
            if self._all_aligned():
                self.started_moving = True
                self.in_goal_count = 0
        
        if self.started_moving:
            if distance > self.stop_dist:
                ang_cmd = self._clamp(self.k_ang * angle_diff, -self.max_angular_speed, self.max_angular_speed)
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
                twist.linear.x = 0.0
                
                if abs(angle_diff) > self.final_yaw_thresh:
                    twist.angular.z = self._clamp(self.k_ang * angle_diff, -self.max_angular_speed, self.max_angular_speed)
                else:
                    twist.angular.z = 0.0
                
                self.in_goal_count += 1
                
                if self.in_goal_count >= self.stop_dwell_cycles:
                    self.current_corner_idx = self.target_corner_idx
                    self.has_goal = False
                    self.started_moving = False
                    self._publish_aligned(False)
                    self.in_goal_count = 0
                    self.get_logger().info(f'{self.robot_name}: Reached corner {self.current_corner_idx}')
        
        self.cmd_vel_pub.publish(twist)
    
    def _nearest_corner_index(self, pose_xy):
        x, y = pose_xy
        best_i, best_d = 0, float('inf')
        for i, (cx, cy) in enumerate(self.corners):
            d = (cx - x) * (cx - x) + (cy - y) * (cy - y)
            if d < best_d:
                best_d, best_i = d, i
        return best_i
    
    @staticmethod
    def _dist2(p, q):
        return (p[0] - q[0])**2 + (p[1] - q[1])**2
    
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

