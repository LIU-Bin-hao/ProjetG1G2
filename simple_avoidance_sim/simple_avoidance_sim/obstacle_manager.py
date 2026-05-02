import rclpy
from rclpy.node import Node
import random
import math
import subprocess

class ObstacleManager(Node):

    def __init__(self):
        super().__init__('obstacle_manager')

        # 参数
        self.num_obstacles = 5
        self.x_range = (5.0, 20.0)
        self.y_range = (-3.0, 3.0)
        self.min_dist = 1.5  # 最小间距

        self.positions = []

        self.spawn_obstacles()

    def is_valid_position(self, x, y):
        # 避开起点 (0,0)
        if math.hypot(x, y) < 3.0:
            return False

        # 避免障碍物重叠
        for (px, py) in self.positions:
            if math.hypot(x - px, y - py) < self.min_dist:
                return False

        return True

    def generate_position(self):
        while True:
            x = random.uniform(*self.x_range)
            y = random.uniform(*self.y_range)

            if self.is_valid_position(x, y):
                self.positions.append((x, y))
                return x, y

    def spawn_obstacles(self):
        self.get_logger().info("Spawning random obstacles...")

        for i in range(self.num_obstacles):
            x, y = self.generate_position()

            name = f"obstacle_{i}"

            cmd = [
                "ros2", "run", "gazebo_ros", "spawn_entity.py",
                "-entity", name,
                "-x", str(x),
                "-y", str(y),
                "-z", "0.5",
                "-file", "/usr/share/gazebo-11/models/box/model.sdf"
            ]

            subprocess.Popen(cmd)

            self.get_logger().info(f"Spawned {name} at ({x:.2f}, {y:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleManager()
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
