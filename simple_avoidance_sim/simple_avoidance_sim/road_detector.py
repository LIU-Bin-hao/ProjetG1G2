import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

import cv2
import numpy as np


class RoadDetector(Node):
    def __init__(self):
        super().__init__('road_detector')

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.error_pub = self.create_publisher(Float32, '/road_center_error', 10)
        self.steer_pub = self.create_publisher(Float32, '/road_steering_correction', 10)

        self.get_logger().info('Road detector started.')

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')
            return

        height, width, _ = frame.shape

        # 只看图像下半部分，更关注前方路面
        roi = frame[int(height * 0.55):, :]

        # 转灰度
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # 轻微模糊，减少噪声
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # 阈值分割：假设道路整体比周围区域更暗
        _, binary = cv2.threshold(blur, 110, 255, cv2.THRESH_BINARY_INV)

        # 形态学处理，填补空洞
        kernel = np.ones((5, 5), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

        # 找轮廓
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        error_msg = Float32()
        steer_msg = Float32()

        if contours:
            # 选面积最大的轮廓，通常认为是道路区域
            largest = max(contours, key=cv2.contourArea)

            area = cv2.contourArea(largest)

            if area > 500:
                M = cv2.moments(largest)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    image_center_x = width // 2

                    # 偏差：道路中心 - 图像中心
                    error = float(cx - image_center_x)

                    # 归一化到 [-1, 1] 左右
                    normalized_error = error / (width / 2.0)

                    # 一个简单比例控制
                    k_p = -0.6
                    steering_correction = k_p * normalized_error

                    # 限幅
                    steering_correction = max(min(steering_correction, 0.4), -0.4)

                    error_msg.data = normalized_error
                    steer_msg.data = steering_correction

                    self.error_pub.publish(error_msg)
                    self.steer_pub.publish(steer_msg)

                    # 调试可视化
                    debug = roi.copy()
                    cv2.drawContours(debug, [largest], -1, (0, 255, 0), 2)
                    cv2.circle(debug, (cx, roi.shape[0] // 2), 5, (0, 0, 255), -1)
                    cv2.line(debug, (image_center_x, 0), (image_center_x, roi.shape[0]), (255, 0, 0), 2)

                    cv2.imshow('road_binary', binary)
                    cv2.imshow('road_debug', debug)
                    cv2.waitKey(1)
                    return

        # 没识别到路时，发布 0
        error_msg.data = 0.0
        steer_msg.data = 0.0
        self.error_pub.publish(error_msg)
        self.steer_pub.publish(steer_msg)

        cv2.imshow('road_binary', binary)
        cv2.imshow('road_debug', roi)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = RoadDetector()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
