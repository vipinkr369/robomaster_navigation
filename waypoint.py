#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from threading import Thread
import tf2_ros
from rclpy.time import Time

class RobotCommand(Node):
    def __init__(self, robot_name: str, command_type: str, coordinates: dict = None, node_id: int = 0):
        super().__init__(f'robot_command_node_{robot_name}_{node_id}')
        self.robot_name = robot_name
        self.command_type = command_type.lower()
        self.coordinates = coordinates or {}

        # TF listener for getting current pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        if self.command_type == 'init':
            self.pub = self.create_publisher(
                PoseWithCovarianceStamped,
                f'/{self.robot_name}/initialpose',
                10
            )
        elif self.command_type == 'goal':
            self.client = ActionClient(
                self,
                NavigateToPose,
                f'/{self.robot_name}/navigate_to_pose'
            )
        else:
            self.get_logger().error(f"Unknown command type: {self.command_type}")

    def execute(self):
        if self.command_type == 'init':
            self.send_initial_pose_from_amcl()
        elif self.command_type == 'goal':
            self.send_goal_pose()

    def get_amcl_pose(self, timeout=5.0):
        """Get the robot's current AMCL pose in map frame."""
        self.get_logger().info("Waiting for AMCL pose...")
        end_time = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        pose_msg = None

        def callback(msg):
            nonlocal pose_msg
            pose_msg = msg

        sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            callback,
            10
        )

        while self.get_clock().now().nanoseconds < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)
            if pose_msg is not None:
                return pose_msg.pose.pose

        self.get_logger().warn("AMCL pose not received, using default (0,0)")
        return PoseWithCovarianceStamped().pose.pose

    def send_initial_pose_from_amcl(self):
        pose = self.get_amcl_pose()
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose = pose
        msg.pose.covariance = [
            0.25,0,0,0,0,0,
            0,0.25,0,0,0,0,
            0,0,0.25,0,0,0,
            0,0,0,0.0685,0,0,
            0,0,0,0,0.0685,0,
            0,0,0,0,0,0.0685
        ]
        self.pub.publish(msg)
        self.get_logger().info(f"Initial pose published from AMCL: x={pose.position.x}, y={pose.position.y}")

    def send_goal_pose(self):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"Action server not available")
            return

        # Use provided coordinates or current pose + offset
        x, y = self.get_current_pose()
        gx = float(self.coordinates.get('x', x + 1.0))
        gy = float(self.coordinates.get('y', y + 1.0))
        gz = float(self.coordinates.get('z', 0.0))
        gw = float(self.coordinates.get('w', 1.0))

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = gx
        pose.pose.position.y = gy
        pose.pose.orientation.z = gz
        pose.pose.orientation.w = gw
        goal_msg.pose = pose

        self.get_logger().info(f"Sending goal: x={gx}, y={gy}")
        send_goal_future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        self.get_logger().info(f"Goal result status: {result.status}")

    def get_current_pose(self, timeout=5.0):
        """Get robot's current pose in map frame via TF"""
        end_time = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while self.get_clock().now().nanoseconds < end_time:
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_link', Time())
                return t.transform.translation.x, t.transform.translation.y
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().warn("TF lookup timed out, using fallback (0,0)")
        return 0.0, 0.0


def run_robot_command(robot_name, command_type, coords=None, node_id=0):
    node = RobotCommand(robot_name, command_type, coords, node_id=node_id)
    node.execute()
    node.destroy_node()


if __name__ == "__main__":
    rclpy.init()

    # Initial pose comes from AMCL
    t1 = Thread(target=run_robot_command, args=('', 'init', None, 1))
    t1.start()
    t1.join()

    # Goal relative to current AMCL pose or offset
    coords_goal = {'x': 2.0, 'y': 3.0}  # absolute in map frame or offset
    t2 = Thread(target=run_robot_command, args=('', 'goal', coords_goal, 2))
    t2.start()
    t2.join()

    rclpy.shutdown()
