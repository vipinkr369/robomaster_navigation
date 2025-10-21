#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from threading import Thread

class RobotCommand(Node):
    def __init__(self, robot_name: str, command_type: str, coordinates: dict, node_id: int = 0):
        # Unique node name for each thread
        super().__init__(f'robot_command_node_{robot_name}_{node_id}')
        self.robot_name = robot_name
        self.command_type = command_type.lower()
        self.coordinates = coordinates

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
            self.send_initial_pose()
        elif self.command_type == 'goal':
            self.send_goal_pose()

    def send_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(self.coordinates.get('x', 0.0))
        msg.pose.pose.position.y = float(self.coordinates.get('y', 0.0))
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = float(self.coordinates.get('z', 0.0))
        msg.pose.pose.orientation.w = float(self.coordinates.get('w', 1.0))
        # Covariance with floats
        msg.pose.covariance = [
            0.25,0.0,0.0,0.0,0.0,0.0,
            0.0,0.25,0.0,0.0,0.0,0.0,
            0.0,0.0,0.25,0.0,0.0,0.0,
            0.0,0.0,0.0,0.0685,0.0,0.0,
            0.0,0.0,0.0,0.0,0.0685,0.0,
            0.0,0.0,0.0,0.0,0.0,0.0685
        ]
        self.pub.publish(msg)
        self.get_logger().info(f"Initial pose published for {self.robot_name}: {self.coordinates}")

    def send_goal_pose(self):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"Action server not available: /{self.robot_name}/navigate_to_pose")
            return

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(self.coordinates.get('x', 0.0))
        pose.pose.position.y = float(self.coordinates.get('y', 0.0))
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = float(self.coordinates.get('z', 0.0))
        pose.pose.orientation.w = float(self.coordinates.get('w', 1.0))
        goal_msg.pose = pose

        send_goal_future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        self.get_logger().info(f"Goal result: {result.status}")


# Thread wrapper function with unique node ID
def run_robot_command(robot_name, command_type, coords, node_id=0):
    node = RobotCommand(robot_name, command_type, coords, node_id=node_id)
    node.execute()
    node.destroy_node()


if __name__ == "__main__":
    rclpy.init()

    # Example: publish initial pose
    coords_init = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
    t1 = Thread(target=run_robot_command, args=('sk01', 'init', coords_init, 1))
    t1.start()

    # Example: send a goal
    coords_goal = {'x': 0.43, 'y': -0.08, 'z': 0.0, 'w': 1.0}
    t2 = Thread(target=run_robot_command, args=('sk01', 'goal', coords_goal, 2))
    t2.start()

    t1.join()
    t2.join()

    rclpy.shutdown()
