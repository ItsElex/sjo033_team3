import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class RoomController(Node):
    def __init__(self):
        super().__init__('room_controller')
        self.get_logger().info('Room controller initialized')
        
        # Publishers and subscribers will go here
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
    
    def odom_callback(self, msg):
        # Handle odometry updates
        pass


def main(args=None):
    rclpy.init(args=args)
    node = RoomController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

