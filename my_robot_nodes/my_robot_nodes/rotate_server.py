#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from my_robot_interfaces.action import Rotate


def yaw_from_quaternion(q):
    """Convert an orientation quaternion into yaw in radians."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def angle_difference(target, current):
    """Return the wrapped angular difference in the range [-pi, pi]."""
    return math.atan2(
        math.sin(target - current),
        math.cos(target - current)
    )


class RotateServer(Node):

    def __init__(self):
        super().__init__('rotate_server')

        self.declare_parameter('turn_speed', 0.5)

        self.yaw = None
        self.group = ReentrantCallbackGroup()

        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.on_odom,
            10,
            callback_group=self.group
        )

        self.action_server = ActionServer(
            self,
            Rotate,
            '/rotate',
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=self.group
        )

        self.get_logger().info('Rotate action server is ready.')

    def on_odom(self, msg):
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def on_goal(self, goal_request):
        if abs(goal_request.angle) < 1e-6:
            self.get_logger().warn('Rejecting zero-angle goal.')
            return GoalResponse.REJECT

        if self.yaw is None:
            self.get_logger().warn('Rejecting goal: no odometry received yet.')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        self.get_logger().info('Rotation cancellation requested.')
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        goal = goal_handle.request.angle
        direction = 1.0 if goal > 0.0 else -1.0

        previous_yaw = self.yaw
        turned = 0.0

        speed = abs(float(self.get_parameter('turn_speed').value))

        twist = Twist()
        twist.angular.z = direction * speed

        feedback = Rotate.Feedback()
        result = Rotate.Result()

        self.get_logger().info(f'Rotating through {goal:.3f} radians.')

        while rclpy.ok():

            if goal_handle.is_cancel_requested:
                self.cmd_vel.publish(Twist())
                result.turned = turned
                goal_handle.canceled()
                self.get_logger().info(
                    f'Rotation canceled after {turned:.3f} radians.'
                )
                return result

            current_yaw = self.yaw

            if current_yaw is not None and previous_yaw is not None:
                # Add the small change between consecutive odometry readings.
                delta = angle_difference(current_yaw, previous_yaw)
                turned += delta
                previous_yaw = current_yaw

            feedback.turned = turned
            goal_handle.publish_feedback(feedback)

            progress = direction * turned

            if progress >= abs(goal):
                break

            self.cmd_vel.publish(twist)
            time.sleep(0.05)

        self.cmd_vel.publish(Twist())

        result.turned = turned
        goal_handle.succeed()

        self.get_logger().info(f'Rotation completed: {turned:.3f} radians.')

        return result


def main():
    rclpy.init()
    node = RotateServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.cmd_vel.publish(Twist())
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
