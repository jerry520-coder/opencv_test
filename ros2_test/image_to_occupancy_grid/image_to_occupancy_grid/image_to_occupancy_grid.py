import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np


class ImageToOccupancyGrid(Node):
    def __init__(self):
        super().__init__("image_to_occupancy_grid")
        self.publisher_ = self.create_publisher(OccupancyGrid, "map", 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.map_resolution = 0.05  # 地图分辨率（每像素的实际尺寸，例如0.05米）
        self.map_origin = [-5.0, -5.0, 0.0]  # 地图的原点

    def timer_callback(self):
        # 读取图像
        image_path = "/home/binghe/orbbec_code/test/picture/新建文件夹(1)/20240427-0501-Alberta-Campos_20240520_142836_00019.jpg"  # 修改为你的图像路径
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

        # 检查图像是否读取成功
        if image is None:
            self.get_logger().error(f"Failed to load image from {image_path}")
            return

        # 将图像二值化
        _, binary_image = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY)

        # 创建占用地图消息
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.frame_id = "map"
        occupancy_grid.info.resolution = self.map_resolution
        occupancy_grid.info.width = binary_image.shape[1]
        occupancy_grid.info.height = binary_image.shape[0]
        occupancy_grid.info.origin.position.x = self.map_origin[0]
        occupancy_grid.info.origin.position.y = self.map_origin[1]
        occupancy_grid.info.origin.position.z = self.map_origin[2]
        occupancy_grid.info.origin.orientation.w = 1.0

        # 填充占用地图数据
        occupancy_grid.data = [0] * (binary_image.shape[0] * binary_image.shape[1])
        for y in range(binary_image.shape[0]):
            for x in range(binary_image.shape[1]):
                pixel = binary_image[y, x]
                # 将像素值转换为占用值：0表示空闲，100表示占用，-1表示未知
                if pixel == 0:
                    occupancy_grid.data[
                        (binary_image.shape[0] - 1 - y) * binary_image.shape[1] + x
                    ] = 100  # 占用黑色
                elif pixel == 255:
                    occupancy_grid.data[
                        (binary_image.shape[0] - 1 - y) * binary_image.shape[1] + x
                    ] = 0  # 空闲白色
                else:
                    occupancy_grid.data[
                        (binary_image.shape[0] - 1 - y) * binary_image.shape[1] + x
                    ] = -1  # 未知

        # 发布占用地图
        self.publisher_.publish(occupancy_grid)


def main(args=None):
    rclpy.init(args=args)
    node = ImageToOccupancyGrid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
