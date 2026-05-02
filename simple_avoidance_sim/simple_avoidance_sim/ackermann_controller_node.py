import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class AckermannControllerNode(Node):
    def __init__(self):
        super().__init__('ackermann_controller_node')

        self.declare_parameter('wheelbase', 0.8)
        self.declare_parameter('wheel_radius', 0.12)
        self.declare_parameter('max_steering_angle', 0.5)

        self.wheelbase = self.get_parameter('wheelbase').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value

        self.steer_pub = self.create_publisher(
            Float64MultiArray,
            '/front_steering_controller/commands',
            10
        )
        self.rear_wheel_pub = self.create_publisher(
            Float64MultiArray,
            '/rear_wheel_controller/commands',
            10
        )

        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.get_logger().info('Ackermann controller node started.')

    def cmd_callback(self, msg: Twist):
        v = msg.linear.x
        omega = msg.angular.z

        steering_angle = 0.0
        if abs(v) > 1e-3 and abs(omega) > 1e-3:
            steering_angle = math.atan(self.wheelbase * omega / v)
        elif abs(omega) > 1e-3:
            steering_angle = self.max_steering_angle if omega > 0 else -self.max_steering_angle

        steering_angle = max(
            -self.max_steering_angle,
            min(self.max_steering_angle, steering_angle)
        )

        wheel_angular_velocity = v / self.wheel_radius

        steer_msg = Float64MultiArray()
        steer_msg.data = [steering_angle, steering_angle]

        drive_msg = Float64MultiArray()
        drive_msg.data = [wheel_angular_velocity, wheel_angular_velocity]

        self.steer_pub.publish(steer_msg)
        self.rear_wheel_pub.publish(drive_msg)

        self.get_logger().info(
            f'v={v:.2f}, omega={omega:.2f}, '
            f'steer={steering_angle:.2f}, wheel_vel={wheel_angular_velocity:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = AckermannControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
