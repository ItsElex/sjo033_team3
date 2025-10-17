import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

class MultiRobotPoseTransformer(Node):
    def __init__(self):
        super().__init__('multi_robot_pose_transformer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.robots = ['robot1', 'robot2', 'robot3', 'robot4']
        # Timer to regularly publish transformed poses
        self.timer = self.create_timer(1.0, self.transform_all)

    def transform_all(self):
        for robot in self.robots:
            target_pose_map = PoseStamped()
            target_pose_map.header.stamp = self.get_clock().now().to_msg()
            target_pose_map.header.frame_id = 'map'
            # Fill this in with the actual target for each robot
            target_pose_map.pose.position.x = 1.0
            target_pose_map.pose.position.y = 2.0
            target_pose_map.pose.position.z = 0.0
            target_pose_map.pose.orientation.w = 1.0

            try:
                transformed = tf2_geometry_msgs.do_transform_pose(
                    target_pose_map,
                    self.tf_buffer.lookup_transform(f'{robot}/odom', 'map', rclpy.time.Time())
                )
                self.get_logger().info(f"{robot}: Goal in odom: {transformed.pose.position.x}, {transformed.pose.position.y}")
                # Publish/use for motion control
            except Exception as e:
                self.get_logger().warn(f'{robot}: No transform found: {e}')

def main():
    rclpy.init()
    node = MultiRobotPoseTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
