import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import os
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from datetime import datetime
from cv_bridge import CvBridge  # 导入cv_bridge来转换ROS图像消息

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')

        # 定义 QoS 配置为 Best Effort
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Reliability setting
            history=HistoryPolicy.KEEP_LAST,  # Keep the last message in the queue
            depth=1  # Set the depth to store the last 10 messages
        )

        # 创建 CvBridge 实例，用于ROS图像消息与OpenCV图像之间的转换
        self.bridge = CvBridge()

        # 定义订阅者，订阅未压缩图像消息
        self.subscription = self.create_subscription(
            Image,
            '/beamforming/beam_overlay_image',  # 替换为你的主题名称
            self.image_callback,
            qos_profile
        )

        # 创建保存图像的文件夹路径
        self.save_folder = '/cae-microphone-array-containerized/src/Extractor_V2/processor/object_tracker/saved_images'
        os.makedirs(self.save_folder, exist_ok=True)

    def image_callback(self, msg):
        try:
            # 使用 CvBridge 将 ROS 图像消息转换为 OpenCV 格式
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # 使用当前时间作为文件名
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            file_path = os.path.join(self.save_folder, f'image_{timestamp}.jpg')
            
            # 保存图像
            cv2.imwrite(file_path, image)
            self.get_logger().info(f'Saved image to {file_path}')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert and save image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    image_saver_node = ImageSaverNode()
    
    try:
        rclpy.spin(image_saver_node)
    except KeyboardInterrupt:
        pass
    finally:
        image_saver_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
