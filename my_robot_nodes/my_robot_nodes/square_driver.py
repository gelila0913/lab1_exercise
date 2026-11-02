#!/usr/bin/env python3

import rclpy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from lab_interfaces.action import DriveDistance
from my_robot_interfaces.action import Rotate


class SquareDriver(Node):

    def __init__(self):
        super().__init__('square_driver')

        self.declare_parameter('side_length', 0.5)
        self.side_length = float(
            self.get_parameter('side_length').value
        )

        self.drive_client = ActionClient(
            self, DriveDistance, '/drive_distance'
        )

        self.rotate_client = ActionClient(
            self, Rotate, '/rotate'
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

    def stop_robot(self):
        self.cmd_vel_publisher.publish(Twist())

    def drive_feedback(self, feedback_msg):
        travelled = feedback_msg.feedback.travelled
        self.get_logger().info(
            f'Drive feedback: {travelled:.2f} m'
        )

    def rotate_feedback(self, feedback_msg):
        turned = feedback_msg.feedback.turned
        self.get_logger().info(
            f'Rotation feedback: {turned:.2f} rad'
        )

    def execute_goal(self, client, goal, feedback_callback, action_name):
        client.wait_for_server()

        send_future = client.send_goal_async(
            goal,
            feedback_callback=feedback_callback
        )

        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                f'{action_name} goal was rejected.'
            )
            return False

        self.get_logger().info(f'{action_name} goal accepted.')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result_response = result_future.result()

        if result_response.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'{action_name} completed successfully.'
            )
            return True

        self.get_logger().error(
            f'{action_name} failed with status '
            f'{result_response.status}.'
        )
        return False

    def run_square(self):
        for side in range(1, 5):

            self.get_logger().info(
                f'========== SIDE {side} =========='
            )

            drive_goal = DriveDistance.Goal()
            drive_goal.distance = self.side_length

            drive_success = self.execute_goal(
                self.drive_client,
                drive_goal,
                self.drive_feedback,
                f'Drive on side {side}'
            )

            if not drive_success:
                self.stop_robot()
                self.get_logger().error(
                    f'Square stopped: drive failed on side {side}.'
                )
                return

            rotate_goal = Rotate.Goal()
            rotate_goal.angle = 1.57079632679

            rotate_success = self.execute_goal(
                self.rotate_client,
                rotate_goal,
                self.rotate_feedback,
                f'Left turn after side {side}'
            )

            if not rotate_success:
                self.stop_robot()
                self.get_logger().error(
                    f'Square stopped: rotation failed after side {side}.'
                )
                return

        self.stop_robot()
        self.get_logger().info(
            'Square completed successfully!'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SquareDriver()

    try:
        node.run_square()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
