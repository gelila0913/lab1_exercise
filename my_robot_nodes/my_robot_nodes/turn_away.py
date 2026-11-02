import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from lab_interfaces.msg import ObstacleInfo
from my_robot_interfaces.msg import RobotStatus
from my_robot_interfaces.srv import SetTurnSpeed


class TurnAway(Node):

    def __init__(self):
        super().__init__('turn_away')

        self.subscription = self.create_subscription(
            ObstacleInfo,
            '/obstacle',
            self.obstacle_callback,
            10
        )

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.status_publisher = self.create_publisher(
            RobotStatus,
            '/robot_status',
            10
        )

        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('max_turn_speed', 1.5)

        self.enabled = False

        self.enable_service = self.create_service(
            SetBool,
            '/turn_away_enable',
            self.enable_callback
        )

        self.speed_service = self.create_service(
            SetTurnSpeed,
            '/set_turn_speed',
            self.set_turn_speed_callback
        )

        self.get_logger().info('Turn Away node started')

    def enable_callback(self, request, response):
        self.enabled = request.data

        if not self.enabled:
            self.publisher.publish(Twist())

            status = RobotStatus()
            status.state = 'stopped'
            status.obstacle_distance = float('inf')
            self.status_publisher.publish(status)

            response.success = True
            response.message = 'Turn Away disabled'

        else:
            response.success = True
            response.message = 'Turn Away enabled'

        self.get_logger().info(response.message)

        return response

    def set_turn_speed_callback(self, request, response):
        max_speed = self.get_parameter('max_turn_speed').value

        if request.turn_speed <= 0 or request.turn_speed > max_speed:
            response.success = False
            response.message = (
                f'Invalid turn speed. Must be greater than 0 '
                f'and at most {max_speed} rad/s.'
            )
        else:
            self.set_parameters([
                rclpy.parameter.Parameter(
                    'turn_speed',
                    rclpy.Parameter.Type.DOUBLE,
                    float(request.turn_speed)
                )
            ])

            response.success = True
            response.message = f'Turn speed updated to {request.turn_speed} rad/s.'

        self.get_logger().info(response.message)
        return response

    def obstacle_callback(self, msg):
        if not self.enabled:
            status = RobotStatus()
            status.state = 'stopped'
            status.obstacle_distance = msg.distance
            self.status_publisher.publish(status)
            return

        forward_speed = self.get_parameter('forward_speed').value
        turn_speed = self.get_parameter('turn_speed').value

        velocity = Twist()

        if not msg.too_close:
            velocity.linear.x = forward_speed
            velocity.angular.z = 0.0
            self.get_logger().info('Path clear: moving forward')

        else:
            velocity.linear.x = 0.0

            if msg.angle > 0:
                velocity.angular.z = -turn_speed
                self.get_logger().info('Obstacle on left: turning right')
            else:
                velocity.angular.z = turn_speed
                self.get_logger().info('Obstacle on right or ahead: turning left')

        self.publisher.publish(velocity)

        status = RobotStatus()
        status.obstacle_distance = msg.distance

        if not msg.too_close:
            status.state = 'forward'
        else:
            status.state = 'turning'

        self.status_publisher.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = TurnAway()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
