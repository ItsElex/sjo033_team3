import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from example_interfaces.msg import Empty
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.action import ActionClient
import math
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs

class MultiRobotRoomNavigator(Node):
    def __init__(self):
        super().__init__('multi_robot_room_navigator')
        
        # TF2 setup for coordinate transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Robots for synchronized alignment
        self.robots = ['robot1', 'robot2', 'robot3', 'robot4']
        
        # Parameters
        self.declare_parameter('robot_name', 'robot1')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        
        # Room centers - each room has one center coordinate
        self.room_centers = [
            (0.35, -0.3),    # Room 1 center
            (-0.7, -0.5),    # Room 2 center
            (-0.4, 0.7),     # Room 3 center
            (0.75, 0.7),     # Room 4 center
        ]
        
        # Initial room assignment: robot1 in room1, robot2 in room2, etc.
        self.robot_to_room_mapping = {
            'robot1': 0,  # Room 1
            'robot2': 1,  # Room 2
            'robot3': 2,  # Room 3
            'robot4': 3,  # Room 4
        }
        
        # State
        self.current_pose = (0.0, 0.0)
        self.current_yaw = 0.0
        self.current_room_idx = self.robot_to_room_mapping[self.robot_name]
        self.target_room_idx = self.current_room_idx
        self.has_goal = False
        self.started_moving = False
        self.map_ready = False
        
        # Alignment flags across robots (for synchronized start)
        self.alignment_statuses = {r: False for r in self.robots}
        
        # QoS for aligned (latched-like)
        aligned_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Subscriptions
        self.create_subscription(Odometry, f'/{self.robot_name}/odom', self.odom_cb, 10)
        self.create_subscription(Empty, '/left', self.left_cb, 10)
        self.create_subscription(Empty, '/right', self.right_cb, 10)
        
        for robot in self.robots:
            self.create_subscription(
                Bool, 
                f'/{robot}/aligned', 
                self._aligned_cb_factory(robot), 
                aligned_qos
            )
        
        # Publishers
        self.aligned_pub = self.create_publisher(Bool, f'/{self.robot_name}/aligned', aligned_qos)
        
        # Nav2 action client
        self.nav_client = ActionClient(
            self, 
            NavigateToPose,
            f'/{self.robot_name}/navigate_to_pose'
        )
        
        # Control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Map readiness check timer
        self.map_check_timer = self.create_timer(1.0, self.check_map_ready)
        
        # Approach and goal-checking tunables
        self.angle_threshold = 0.12
        self.final_yaw_thresh = 0.15
        self.stop_dwell_cycles = 8
        self.in_goal_count = 0
        
        # Motion gains and limits for alignment rotation
        self.max_angular_speed = 0.6
        self.k_ang = 2.0
        
        self.get_logger().info(
            f'{self.robot_name}: Room navigator initialized in room {self.current_room_idx} '
            f'at {self.room_centers[self.current_room_idx]}'
        )
    
    def check_map_ready(self):
        """Check if map frame is available"""
        try:
            self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            if not self.map_ready:
                self.map_ready = True
                self.get_logger().info(f'{self.robot_name}: Map frame is now ready!')
        except TransformException:
            self.map_ready = False
    
    def odom_cb(self, msg: Odometry):
        """Update current pose from odometry"""
        try:
            # Skip if map frame not ready yet
            if not self.map_ready:
                return
            
            pose_in_odom = PoseStamped()
            pose_in_odom.header = msg.header
            pose_in_odom.pose = msg.pose.pose
            
            pose_in_map = self.tf_buffer.transform(
                pose_in_odom,
                'map',
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            self.current_pose = (
                pose_in_map.pose.position.x, 
                pose_in_map.pose.position.y
            )
            
            q = pose_in_map.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
            
        except TransformException as ex:
            # Silently ignore - map not ready yet
            return
    
    def left_cb(self, _):
        """Move to previous room"""
        self._new_command(direction=-1)
    
    def right_cb(self, _):
        """Move to next room"""
        self._new_command(direction=+1)
    
    def _new_command(self, direction: int):
        """Handle left/right movement command"""
        num_rooms = len(self.room_centers)
        
        # Calculate target room with proper wraparound
        self.target_room_idx = (self.current_room_idx + direction) % num_rooms
        
        # Reset alignment flags
        for k in self.alignment_statuses:
            self.alignment_statuses[k] = False
        self._publish_aligned(False)
        
        # Reset control state
        self.has_goal = True
        self.started_moving = False
        self.in_goal_count = 0
        
        self.get_logger().info(
            f'{self.robot_name}: Moving from room {self.current_room_idx} to '
            f'room {self.target_room_idx} (direction={direction})'
        )
    
    def _aligned_cb_factory(self, robot):
        """Factory for creating aligned callbacks for each robot"""
        def cb(msg: Bool):
            self.alignment_statuses[robot] = bool(msg.data)
        return cb
    
    def control_loop(self):
        """Main control loop - handle alignment and navigation"""
        if not self.has_goal or not self.map_ready:
            self._publish_aligned(False)
            return
        
        x, y = self.current_pose
        tx, ty = self.room_centers[self.target_room_idx]
        
        dx, dy = (tx - x), (ty - y)
        target_angle = math.atan2(dy, dx)
        angle_diff = self._norm_ang(target_angle - self.current_yaw)
        
        # Alignment phase: rotate to face target
        if not self.started_moving:
            if abs(angle_diff) > self.angle_threshold:
                # Still rotating to align
                self._publish_aligned(False)
                return
            
            # Robot is aligned
            self.alignment_statuses[self.robot_name] = True
            self._publish_aligned(True)
            
            # Check if all robots are aligned
            if self._all_aligned():
                self.started_moving = True
                self.in_goal_count = 0
                self._send_nav2_goal(tx, ty)
                self.get_logger().info(
                    f'{self.robot_name}: All robots aligned, sending Nav2 goal to room {self.target_room_idx}'
                )
        
        # Movement phase: let Nav2 handle it, monitor completion
        if self.started_moving:
            # Check if we've reached the goal (simple distance check)
            distance = math.hypot(dx, dy)
            
            if distance < 0.15:  # Goal threshold
                self.in_goal_count += 1
                
                if self.in_goal_count >= self.stop_dwell_cycles:
                    self.current_room_idx = self.target_room_idx
                    self.has_goal = False
                    self.started_moving = False
                    self._publish_aligned(False)
                    self.in_goal_count = 0
                    self.get_logger().info(
                        f'{self.robot_name}: Reached room {self.current_room_idx}'
                    )
            else:
                self.in_goal_count = 0
    
    def _send_nav2_goal(self, x: float, y: float):
        """Send goal to Nav2 for navigation"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position = Point(x=x, y=y, z=0.0)
        goal_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        self.nav_client.send_goal_async(goal_msg)
    
    def _all_aligned(self) -> bool:
        """Check if all robots have published aligned=True"""
        return all(self.alignment_statuses.values())
    
    def _publish_aligned(self, aligned: bool):
        """Publish alignment status"""
        self.aligned_pub.publish(Bool(data=aligned))
    
    @staticmethod
    def _clamp(x, lo, hi):
        return max(lo, min(hi, x))
    
    @staticmethod
    def _norm_ang(a):
        """Normalize angle to [-pi, pi]"""
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotRoomNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

