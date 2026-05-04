#!/usr/bin/env python3

import math
from enum import Enum

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class DrivingState(Enum):
    CRUISE = 1
    SLOW_DOWN = 2
    STOP = 3


class TownLaserAvoidanceNode(Node):
    def __init__(self):
        super().__init__("town_obstacle_avoidance_node")

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("road_center_error_topic", "/road/center_error")
        self.declare_parameter("road_steer_gain", 0.8)

        self.declare_parameter("control_frequency", 20.0)
        
        self.declare_parameter("self_ignore_distance", 0.35)

        # Distance thresholds, unit: meter
        self.declare_parameter("slow_down_distance", 5.0)
        self.declare_parameter("stop_distance", 1.5)

        # Speed settings
        self.declare_parameter("cruise_speed", 1.2)
        self.declare_parameter("slow_speed", 0.4)

        # Turning settings
        self.declare_parameter("turn_speed", 0.6)
        self.declare_parameter("avoidance_turn_gain", 0.8)
        self.declare_parameter("max_angular_z", 0.8)

        # Laser sector settings, unit: degree
        self.declare_parameter("front_angle_deg", 20.0)
        self.declare_parameter("side_angle_min_deg", 20.0)
        self.declare_parameter("side_angle_max_deg", 70.0)

        # Safety timeout
        self.declare_parameter("scan_timeout", 0.5)

        self.scan_topic = self.get_parameter("scan_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        
        self.road_center_error_topic = self.get_parameter("road_center_error_topic").value
        self.road_steer_gain = self.get_parameter("road_steer_gain").value

        self.control_frequency = self.get_parameter("control_frequency").value

        self.slow_down_distance = self.get_parameter("slow_down_distance").value
        self.stop_distance = self.get_parameter("stop_distance").value

        self.cruise_speed = self.get_parameter("cruise_speed").value
        self.slow_speed = self.get_parameter("slow_speed").value

        self.turn_speed = self.get_parameter("turn_speed").value
        self.avoidance_turn_gain = self.get_parameter("avoidance_turn_gain").value
        self.max_angular_z = self.get_parameter("max_angular_z").value

        self.front_angle_deg = self.get_parameter("front_angle_deg").value
        self.side_angle_min_deg = self.get_parameter("side_angle_min_deg").value
        self.side_angle_max_deg = self.get_parameter("side_angle_max_deg").value

        self.scan_timeout = self.get_parameter("scan_timeout").value
        self.self_ignore_distance = self.get_parameter("self_ignore_distance").value

        # -----------------------------
        # Runtime variables
        # -----------------------------
        self.latest_scan = None
        self.last_scan_time = self.get_clock().now()

        self.road_center_error = 0.0
        self.last_road_error_time = self.get_clock().now()

        self.state = DrivingState.STOP

        # -----------------------------
        # ROS interfaces
        # -----------------------------
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )
        
        self.road_error_sub = self.create_subscription(
            Float32,
            self.road_center_error_topic,
            self.road_error_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        timer_period = 1.0 / self.control_frequency
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info("Town laser avoidance node started.")
        self.get_logger().info(f"Subscribing scan topic: {self.scan_topic}")
        self.get_logger().info(f"Publishing cmd_vel topic: {self.cmd_vel_topic}")

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg
        self.last_scan_time = self.get_clock().now()

    def control_loop(self):
        if not self.scan_is_alive():
            self.publish_stop()
            self.state = DrivingState.STOP
            self.get_logger().warn(
                "No recent /scan data. Stop vehicle.",
                throttle_duration_sec=1.0
            )
            return

        front_dist = self.get_sector_min_distance(
            self.latest_scan,
            -self.front_angle_deg,
            self.front_angle_deg
        )

        left_dist = self.get_sector_min_distance(
            self.latest_scan,
            self.side_angle_min_deg,
            self.side_angle_max_deg
        )

        right_dist = self.get_sector_min_distance(
            self.latest_scan,
            -self.side_angle_max_deg,
            -self.side_angle_min_deg
        )

        self.state = self.update_state(front_dist)

        speed, angular_z = self.compute_cmd(
            self.state,
            front_dist,
            left_dist,
            right_dist
        )

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"state={self.state.name}, "
            f"front={front_dist:.2f}, "
            f"left={left_dist:.2f}, "
            f"right={right_dist:.2f}, "
            f"road_error={self.road_center_error:.2f}, "
            f"v={speed:.2f}, "
            f"w={angular_z:.2f}",
            throttle_duration_sec=0.5
        )

    def scan_is_alive(self) -> bool:
        if self.latest_scan is None:
            return False

        now = self.get_clock().now()
        dt = (now - self.last_scan_time).nanoseconds / 1e9

        return dt <= self.scan_timeout

    def update_state(self, front_dist: float) -> DrivingState:
        if front_dist <= self.stop_distance:
            return DrivingState.STOP
        elif front_dist <= self.slow_down_distance:
            return DrivingState.SLOW_DOWN
        else:
            return DrivingState.CRUISE

    def compute_cmd(
        self,
        state: DrivingState,
        front_dist: float,
        left_dist: float,
        right_dist: float
    ):
        road_steer = - self.road_steer_gain * self.road_center_error
        road_steer = self.clamp(
            road_steer,
            -self.max_angular_z,
            self.max_angular_z
        )

        if state == DrivingState.CRUISE:
            speed = self.cruise_speed
            angular_z = road_steer

        elif state == DrivingState.SLOW_DOWN:
            speed = self.slow_speed

            road_edge_threshold = 0.45

            # 已经偏离道路中心较多时，优先沿黑色道路回中
            if abs(self.road_center_error) > road_edge_threshold:
                angular_z = road_steer
            else:
                # 只在车还比较靠近道路中心时允许避障偏置
                side_diff = left_dist - right_dist
                avoidance_steer = self.avoidance_turn_gain * side_diff
                avoidance_steer = self.clamp(
                    avoidance_steer,
                    -self.max_angular_z,
                    self.max_angular_z
                )

                angular_z = road_steer + avoidance_steer

            angular_z = self.clamp(
                angular_z,
                -self.max_angular_z,
                self.max_angular_z
            )

        else:
            speed = 0.0
            angular_z = 0.0

        return speed, angular_z
    
    def get_sector_min_distance(
        self,
        scan: LaserScan,
        min_angle_deg: float,
        max_angle_deg: float
    ) -> float:
        min_angle = math.radians(min_angle_deg)
        max_angle = math.radians(max_angle_deg)

        valid_distances = []

        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment

            if min_angle <= angle <= max_angle:
                if self.is_valid_range(scan, r):
                    valid_distances.append(r)

        if len(valid_distances) == 0:
            return scan.range_max

        return min(valid_distances)

    def is_valid_range(self, scan: LaserScan, r: float) -> bool:
        if math.isnan(r):
            return False

        if math.isinf(r):
            return False

        if r < max(scan.range_min, self.self_ignore_distance):
            return False

        if r > scan.range_max:
            return False

        return True

    def publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    @staticmethod
    def clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(value, max_value))
        
    def road_error_callback(self, msg: Float32):
        self.road_center_error = msg.data
        self.last_road_error_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)

    node = TownLaserAvoidanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt. Stop vehicle.")
        node.publish_stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
