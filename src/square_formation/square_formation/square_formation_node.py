#!/usr/bin/env python3
import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from example_interfaces.msg import Empty
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs  # needed for PoseStamped transforms


class SquareFormationNode(Node):
    def __init__(self):
        # Get robot name from CLI args BEFORE creating node name
        robot_name = 'robot1'
        for arg in sys.argv:
            if 'robot_name:=' in arg:
                robot_name = arg.split('robot_name:=', 1)[1]
                break

        super().__init__(f'square_formation_{robot_name}')
        self.robot_name = robot_name

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Robots list
        self.robots = ['robot1', 'robot2', 'robot3', 'robot4']

        # Square corners (global map frame)
        self.corners = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]

        # State
        self.current_pose = None
        self.current_yaw = 0.0
        self.current_corner_idx = 0
        self.target_corner_idx = 0
        self.target_pose = self.corners[self.target_corner_idx]
        self.has_goal = False
        self.started_moving = False
        self.desired_yaw = 0.0

        # Alignment tracking
        # alignment_statuses: bool for convenience (True if last message was True)
        # alignment_counters: integer counter used to require freshness/stability
        self.alignment_statuses = {r: False for r in self.robots}
        self.alignment_counters = {r: 0 for r in self.robots}
        # How many consecutive control cycles a True must be seen to count as stable
        self.required_align_cycles = 6  

        # QoS for aligned (transient-local so late-joiners can see last value)
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
        self.timer_dt = 0.1
        self.timer = self.create_timer(self.timer_dt, self.control_loop)

        # Approach & motion tunables
        self.angle_threshold = 0.15
        self.stop_dist = 0.06
        self.final_yaw_thresh = 0.15
        self.stop_dwell_cycles = 8
        self.in_goal_count = 0

        self.max_angular_speed = 0.8
        self.v_max = 0.2
        self.k_ang = 1.5
        self.k_lin = 1.0
        self.slowdown_dist = 0.40
        self.v_near_max = 0.1
        self.v_min = 0.05

        self.snap_tol = 0.12

        self.get_logger().info(f'{self.robot_name}: Square formation node initialized')

    def odom_cb(self, msg: Odometry):
        try:
            pose_in_odom = PoseStamped()
            pose_in_odom.header = msg.header
            pose_in_odom.pose = msg.pose.pose

            pose_in_map = self.tf_buffer.transform(
                pose_in_odom,
                'map',
                timeout=Duration(seconds=1.0)
            )

            self.current_pose = (pose_in_map.pose.position.x, pose_in_map.pose.position.y)

            q = pose_in_map.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        except TransformException as ex:
            self.get_logger().warn(f'{self.robot_name}: Could not transform odom to map: {ex}')
            return

    def left_cb(self, _):
        if self.has_goal:
            return
        self._new_command(direction=-1)

    def right_cb(self, _):
        if self.has_goal:
            return
        self._new_command(direction=+1)

    def _new_command(self, direction: int):
        # Need a valid pose to choose nearest corner reliably
        if self.current_pose is None:
            self.get_logger().warn(f'{self.robot_name}: Ignoring move command - no valid pose yet')
            return

        nearest_idx = self._nearest_corner_index(self.current_pose)
        if self._dist2(self.current_pose, self.corners[nearest_idx]) <= self.snap_tol * self.snap_tol:
            self.current_corner_idx = nearest_idx

        self.target_corner_idx = (self.current_corner_idx + direction) % len(self.corners)
        self.target_pose = self.corners[self.target_corner_idx]

        # Reset alignment info for ALL robots (clears stale True values)
        for k in self.alignment_statuses:
            self.alignment_statuses[k] = False
            self.alignment_counters[k] = 0
        # Explicitly clear our own contact too
        self.alignment_statuses[self.robot_name] = False
        self.alignment_counters[self.robot_name] = 0
        self._publish_aligned(False)

        # Store current yaw as desired formation heading
        self.desired_yaw = self.current_yaw

        self.has_goal = True
        self.started_moving = False
        self.in_goal_count = 0

        self.get_logger().info(f'{self.robot_name}: Commanded to move to corner {self.target_corner_idx} (target: {self.target_pose})')

    def _aligned_cb_factory(self, robot):
        def cb(msg: Bool):
            # When we receive aligned==True, set that robot's counter to required_align_cycles.
            # This makes sure alignment is fresh and stable (not stale).
            if msg.data:
                self.alignment_statuses[robot] = True
                self.alignment_counters[robot] = self.required_align_cycles
            else:
                # explicit False -> clear status and counter
                self.alignment_statuses[robot] = False
                self.alignment_counters[robot] = 0
        return cb

    def control_loop(self):
        twist = Twist()

        # First: age alignment counters (freshness)
        for r in self.robots:
            if self.alignment_counters[r] > 0:
                # keep marked as True while counter positive
                # decrement each cycle
                self.alignment_counters[r] -= 1
                if self.alignment_counters[r] == 0:
                    # counter expired -> mark as not aligned
                    self.alignment_statuses[r] = False

        if not self.has_goal:
            # publish False so others know we're idle
            self.alignment_statuses[self.robot_name] = False
            self._publish_aligned(False)
            self.cmd_vel_pub.publish(twist)
            return

        if self.current_pose is None:
            self.get_logger().warn(f'{self.robot_name}: waiting for first valid pose in map frame')
            self.cmd_vel_pub.publish(twist)
            return

        x, y = self.current_pose
        tx, ty = self.target_pose
        dx, dy = (tx - x), (ty - y)
        distance = math.hypot(dx, dy)

        if distance < 1e-6:
            angle_diff = 0.0
        else:
            target_angle = math.atan2(dy, dx)
            angle_diff = self._norm_ang(target_angle - self.current_yaw)

        # Alignment phase before move
        if not self.started_moving:
            if abs(angle_diff) > self.angle_threshold:
                # rotate in place, mark ourselves as not aligned and publish False
                twist.linear.x = 0.0
                twist.angular.z = self._clamp(self.k_ang * angle_diff, -self.max_angular_speed, self.max_angular_speed)
                self.alignment_statuses[self.robot_name] = False
                self.alignment_counters[self.robot_name] = 0
                self._publish_aligned(False)
                self.cmd_vel_pub.publish(twist)
                return

            # We are facing the travel direction -> declare aligned and publish a fresh True
            self.alignment_statuses[self.robot_name] = True
            self.alignment_counters[self.robot_name] = self.required_align_cycles
            self._publish_aligned(True)

            # Only start moving when every robot's alignment_status is True (and therefore fresh)
            if self._all_aligned():
                self.desired_yaw = self.current_yaw
                self.started_moving = True
                self.in_goal_count = 0
                self.get_logger().info(f'{self.robot_name}: All aligned - starting move to corner {self.target_corner_idx}')
                # continue to movement logic in same cycle

        # Movement / approach
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
                # close enough to position -> stop forward and align to desired yaw
                twist.linear.x = 0.0
                final_angle_diff = self._norm_ang(self.desired_yaw - self.current_yaw)
                if abs(final_angle_diff) > self.final_yaw_thresh:
                    twist.angular.z = self._clamp(self.k_ang * final_angle_diff, -self.max_angular_speed, self.max_angular_speed)
                else:
                    twist.angular.z = 0.0

                # mark ourselves as aligned while finishing (fresh)
                self.alignment_statuses[self.robot_name] = True
                self.alignment_counters[self.robot_name] = self.required_align_cycles
                self._publish_aligned(True)

                self.in_goal_count += 1

                if self.in_goal_count >= self.stop_dwell_cycles:
                    # reached and stabilized
                    self.current_corner_idx = self.target_corner_idx
                    self.has_goal = False
                    self.started_moving = False
                    # clear and publish False so next command must re-synchronize
                    for k in self.alignment_statuses:
                        self.alignment_statuses[k] = False
                        self.alignment_counters[k] = 0
                    self._publish_aligned(False)
                    self.in_goal_count = 0
                    self.get_logger().info(f'{self.robot_name}: Reached corner {self.current_corner_idx}')

        self.cmd_vel_pub.publish(twist)

    def _nearest_corner_index(self, pose_xy):
        if pose_xy is None:
            return 0
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
        # All entries must be True (and counters ensure freshness)
        return all(self.alignment_statuses.values())

    def _publish_aligned(self, aligned: bool):
        try:
            self.aligned_pub.publish(Bool(data=aligned))
        except Exception as e:
            self.get_logger().warn(f'{self.robot_name}: Failed to publish aligned status: {e}')

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
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

