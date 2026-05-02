import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # 距离阈值
        self.stop_distance = 0.8
        self.slow_distance = 1.5

        # 速度参数
        self.fast_speed = 0.8
        self.slow_speed = 0.35
        self.creep_speed = 0.15

        # 转向参数（给 Ackermann 控制器的角速度参考）
        self.max_turn = 0.8
        self.medium_turn = 0.45

        self.get_logger().info('Obstacle avoidance node started.')

    def clean_values(self, values):
        valid = [
            v for v in values
            if not math.isinf(v) and not math.isnan(v) and v > 0.20
        ]
        if len(valid) == 0:
            return [999.0]
        return valid

    def min_distance(self, values):
        return min(self.clean_values(values))

    def mean_distance(self, values):
        clean = self.clean_values(values)
        return sum(clean) / len(clean)

    def scan_callback(self, msg: LaserScan):
        ranges = list(msg.ranges)
        n = len(ranges)

        if n < 60:
            self.get_logger().warn('Laser scan data too short.')
            return

        center = n // 2

        # 中间更窄一些，更像“前方碰撞检查”
        front_ranges = ranges[center - 25:center + 25]

        # 左右各取一段
        left_ranges = ranges[center + 30:center + 120]
        right_ranges = ranges[center - 120:center - 30]

        front_min = self.min_distance(front_ranges)
        left_mean = self.mean_distance(left_ranges)
        right_mean = self.mean_distance(right_ranges)

        cmd = Twist()

        # 计算左右转向偏置：左边更空 -> 正值；右边更空 -> 负值
        steer_bias = left_mean - right_mean

        # 限幅前的比例控制
        turn_cmd = 0.9 * steer_bias
        turn_cmd = max(-self.max_turn, min(self.max_turn, turn_cmd))

        if front_min > self.slow_distance:
            # 前方很安全：快速前进，小幅修正方向
            cmd.linear.x = self.fast_speed
            cmd.angular.z = max(-self.medium_turn, min(self.medium_turn, turn_cmd))

        elif front_min > self.stop_distance:
            # 前方开始接近障碍：减速并增强转向
            cmd.linear.x = self.slow_speed
            cmd.angular.z = turn_cmd

        else:
            # 前方很近：低速强转向
            cmd.linear.x = self.creep_speed

            if abs(steer_bias) < 0.08:
                # 左右差不多时，固定选一个方向，避免抖动
                cmd.angular.z = self.max_turn
            else:
                cmd.angular.z = turn_cmd

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'front={front_min:.2f}, '
            f'left_mean={left_mean:.2f}, '
            f'right_mean={right_mean:.2f}, '
            f'cmd=({cmd.linear.x:.2f}, {cmd.angular.z:.2f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
