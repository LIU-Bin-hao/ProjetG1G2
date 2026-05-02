import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates


class GoalNavigationNode(Node):
    def __init__(self):
        super().__init__('goal_navigation_node')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )

        # 目标点
        self.goal_x = -4.5
        self.goal_y = 0.0

        # 车辆模型名
        self.vehicle_name = 'simple_car'

        # 当前位置
        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        # 雷达状态
        self.front_min = 999.0
        self.left_mean = 999.0
        self.right_mean = 999.0

        # 参数
        self.goal_tolerance = 0.5
        self.stop_distance = 1.2
        self.slow_distance = 2.0

        self.fast_speed = 0.9
        self.slow_speed = 0.45
        self.creep_speed = 0.20

        self.max_turn = 0.8
        self.nav_turn_gain = 1.2
        self.avoid_turn_gain = 0.9

        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Goal navigation node started.')

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

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
            return

        center = n // 2

        front_ranges = ranges[center - 25:center + 25]
        left_ranges = ranges[center + 30:center + 120]
        right_ranges = ranges[center - 120:center - 30]

        self.front_min = self.min_distance(front_ranges)
        self.left_mean = self.mean_distance(left_ranges)
        self.right_mean = self.mean_distance(right_ranges)

    def model_states_callback(self, msg: ModelStates):
        if self.vehicle_name not in msg.name:
            return

        idx = msg.name.index(self.vehicle_name)
        pose = msg.pose[idx]

        self.current_x = pose.position.x
        self.current_y = pose.position.y
        self.current_yaw = self.quaternion_to_yaw(pose.orientation)

    def control_loop(self):
        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            return

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        goal_dist = math.hypot(dx, dy)

        cmd = Twist()

        # 到达目标
        if goal_dist < self.goal_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.get_logger().info(
                f'Goal reached at ({self.current_x:.2f}, {self.current_y:.2f})'
            )
            return

        goal_heading = math.atan2(dy, dx)
        heading_error = self.normalize_angle(goal_heading - self.current_yaw)

        nav_turn = self.nav_turn_gain * heading_error
        nav_turn = max(-self.max_turn, min(self.max_turn, nav_turn))

        # 左侧更空 -> steer_bias 为正
        steer_bias = self.left_mean - self.right_mean
        avoid_turn = self.avoid_turn_gain * steer_bias
        avoid_turn = max(-self.max_turn, min(self.max_turn, avoid_turn))

        if self.front_min > self.slow_distance:
            # 正常朝目标走
            cmd.linear.x = self.fast_speed
            cmd.angular.z = nav_turn

        elif self.front_min > self.stop_distance:
            # 边朝目标，边避障
            cmd.linear.x = self.slow_speed
            mixed_turn = 0.4 * nav_turn + 0.6 * avoid_turn
            cmd.angular.z = max(-self.max_turn, min(self.max_turn, mixed_turn))

        else:
            # 障碍很近，避障优先
            cmd.linear.x = self.creep_speed

            if abs(steer_bias) < 0.08:
                cmd.angular.z = self.max_turn
            else:
                cmd.angular.z = avoid_turn

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'pos=({self.current_x:.2f},{self.current_y:.2f}) '
            f'goal=({self.goal_x:.2f},{self.goal_y:.2f}) '
            f'd={goal_dist:.2f} '
            f'front={self.front_min:.2f} '
            f'nav_turn={nav_turn:.2f} '
            f'avoid_turn={avoid_turn:.2f} '
            f'cmd=({cmd.linear.x:.2f},{cmd.angular.z:.2f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GoalNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
