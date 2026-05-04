#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge


class BlackRoadFollowerNode(Node):
    def __init__(self):
        super().__init__("black_road_follower_node")

        self.declare_parameter("image_topic", "/camera/front_camera/image_raw")
        self.declare_parameter("center_error_topic", "/road/center_error")

        # 图像下半部分更接近车前道路，所以只处理 ROI
        self.declare_parameter("roi_start_ratio", 0.55)
        self.declare_parameter("roi_end_ratio", 0.95)

        # 黑色道路阈值，越小越“黑”
        self.declare_parameter("black_threshold", 60)

        # 最小道路面积，太小则认为没看到路
        self.declare_parameter("min_road_area", 500)

        self.image_topic = self.get_parameter("image_topic").value
        self.center_error_topic = self.get_parameter("center_error_topic").value

        self.roi_start_ratio = self.get_parameter("roi_start_ratio").value
        self.roi_end_ratio = self.get_parameter("roi_end_ratio").value
        self.black_threshold = self.get_parameter("black_threshold").value
        self.min_road_area = self.get_parameter("min_road_area").value

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.error_pub = self.create_publisher(
            Float32,
            self.center_error_topic,
            10
        )

        self.get_logger().info("Black road follower node started.")
        self.get_logger().info(f"Subscribing image: {self.image_topic}")
        self.get_logger().info(f"Publishing road center error: {self.center_error_topic}")

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"Failed to convert image: {e}")
            return

        height, width, _ = frame.shape

        roi_y1 = int(height * self.roi_start_ratio)
        roi_y2 = int(height * self.roi_end_ratio)

        roi = frame[roi_y1:roi_y2, :]

        # 转灰度
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # 黑色道路 mask：灰度值越低越黑
        road_mask = cv2.inRange(gray, 0, int(self.black_threshold))

        # 去噪
        kernel = np.ones((5, 5), np.uint8)
        road_mask = cv2.morphologyEx(road_mask, cv2.MORPH_OPEN, kernel)
        road_mask = cv2.morphologyEx(road_mask, cv2.MORPH_CLOSE, kernel)

        moments = cv2.moments(road_mask)

        if moments["m00"] < self.min_road_area:
            # 没看到足够大的黑色道路，先发布 0，让车保持当前方向
            error_msg = Float32()
            error_msg.data = 0.0
            self.error_pub.publish(error_msg)

            self.get_logger().warn(
                "Black road not detected clearly. Publishing center_error=0.0",
                throttle_duration_sec=1.0
            )
            return

        road_center_x = int(moments["m10"] / moments["m00"])
        image_center_x = width // 2

        # 归一化误差，范围大概是 -1 到 1
        # road_center_x > image_center_x：道路中心在图像右边，车应该向右转
        center_error = (road_center_x - image_center_x) / float(image_center_x)

        error_msg = Float32()
        error_msg.data = float(center_error)
        self.error_pub.publish(error_msg)

        self.get_logger().info(
            f"road_center_x={road_center_x}, image_center_x={image_center_x}, "
            f"center_error={center_error:.3f}",
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)

    node = BlackRoadFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
