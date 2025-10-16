import math
from functools import partial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from example_interfaces.msg import Empty
from std_msgs.msg import Bool

class SquareFormationNode(Node):
    def __init__(self):
        super().__init__('square_formation_node')
        self.robots = ['robot1','robot2','robot3','robot4']
        self.positions = {'robot1':(0,0),'robot2':(1,0),'robot3':(1,1),'robot4':(0,1)}
        self.declare_parameter('robot_name','robot1')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        self.target_pose = self.positions[self.robot_name]
        self.current_pose = (0.0,0.0); self.current_yaw = 0.0
        self.has_goal = False; self.started_moving = False
        self.alignment_statuses = dict.fromkeys(self.robots, False)
        self.reached_statuses   = dict.fromkeys(self.robots, False)

        aligned_qos = QoSProfile(depth=1,
                                reliability=QoSReliabilityPolicy.RELIABLE,
                                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(Odometry, f'/{self.robot_name}/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', 10)

        self.create_subscription(Empty, '/left', partial(self.neighbor_move, -1), 10)
        self.create_subscription(Empty, '/right', partial(self.neighbor_move, +1), 10)

        for r in self.robots:
            self.create_subscription(Bool, f'/{r}/aligned',  partial(self._state_cb, self.alignment_statuses, r), aligned_qos)
            self.create_subscription(Bool, f'/{r}/reached', partial(self._state_cb, self.reached_statuses,   r), 10)

        self.aligned_pub = self.create_publisher(Bool, f'/{self.robot_name}/aligned', aligned_qos)
        self.reached_pub = self.create_publisher(Bool, f'/{self.robot_name}/reached', 10)
        self._aligned_msg = Bool(); self._reached_msg = Bool()

        self.angle_threshold = 0.15; self.dist_threshold = 0.10
        self.max_angular_speed = 0.6; self.forward_speed = 0.1

        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg: Odometry):
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny = 2*(q.w*q.z + q.x*q.y); cosy = 1 - 2*(q.y*q.y + q.z*q.z)
        self.current_yaw = math.atan2(siny, cosy)

    def neighbor_move(self, direction:int, _msg=None):
        idx = self.robots.index(self.robot_name)
        neighbor = self.robots[(idx + direction) % len(self.robots)]
        self.target_pose = self.positions[neighbor]
        self.has_goal = True; self.started_moving = False
        self.alignment_statuses = dict.fromkeys(self.robots, False)
        self.reached_statuses = dict.fromkeys(self.robots, False)
        self.publish_aligned(False); self.publish_reached(False)
        self.get_logger().info(f'{self.robot_name}: {"LEFT" if direction<0 else "RIGHT"} -> {neighbor} {self.target_pose}')

    def _state_cb(self, state_dict, key, msg: Bool):
        state_dict[key] = bool(msg.data)

    def control_loop(self):
        t = Twist()
        if not self.has_goal:
            self.publish_aligned(False); self.cmd_vel_pub.publish(t); return

        x,y = self.current_pose; tx,ty = self.target_pose
        dx,dy = tx-x, ty-y
        target_angle = math.atan2(dy, dx)
        angle_diff = ((target_angle - self.current_yaw) + math.pi) % (2*math.pi) - math.pi
        dist = math.hypot(dx, dy)

        if not self.started_moving:
            if abs(angle_diff) > self.angle_threshold:
                t.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, 2.0 * angle_diff))
                self.publish_aligned(False); self.cmd_vel_pub.publish(t); return
            self.publish_aligned(True)
            if all(self.alignment_statuses.values()):
                self.started_moving = True

        if self.started_moving:
            if dist > self.dist_threshold:
                t.linear.x = self.forward_speed
            else:
                if not self.reached_statuses[self.robot_name]:
                    self.reached_statuses[self.robot_name] = True
                    self.publish_reached(True)
                if all(self.reached_statuses.values()):
                    self.has_goal = False; self.started_moving = False
                    self.publish_aligned(False)
                    t = Twist()

        self.cmd_vel_pub.publish(t)

    def publish_aligned(self, b: bool):
        self._aligned_msg.data = b; self.aligned_pub.publish(self._aligned_msg)

    def publish_reached(self, b: bool):
        self._reached_msg.data = b; self.reached_pub.publish(self._reached_msg)

def main():
    rclpy.init(); n = SquareFormationNode(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()

