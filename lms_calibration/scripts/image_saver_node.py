#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import os
from datetime import datetime


class ImageViewer(Node):
    def __init__(self):
        super().__init__('image_viewer')
        self.bridge = CvBridge()
        self.latest_image = None

        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            10)

        date_str = datetime.now().strftime("%Y%m%d")

        # Create dataset directory if it doesn't exist
        # self.output_dir = os.path.join(os.getcwd(), 'dataset')
        self.output_dir = f"/home/ep51lon/Universe/Workspace/ITB/Research/02_LUMOS/lumos_ws/src/LUMOS/lms_dataset/REC_{date_str}"
        os.makedirs(self.output_dir, exist_ok=True)

        self.get_logger().info("ImageViewer node started. Press SPACE to save image.")

        self.img_counter = 0

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Camera1 Stream", self.latest_image)

            key = cv2.waitKey(1) & 0xFF
            if key == 32:  # SPACE key
                self.img_counter += 1
                self.save_image()

        except Exception as e:
            self.get_logger().error(f"Error displaying or saving image: {e}")

    def save_image(self):
        if self.latest_image is not None:
            date_str = datetime.now().strftime("%Y%m%d")
            time_str = datetime.now().strftime("%H%M%S")
            filename = f"IMG_{date_str}_{time_str}.png"
            filepath = os.path.join(self.output_dir, filename)
            cv2.imwrite(filepath, self.latest_image)
            self.get_logger().info(f"Image {self.img_counter} saved to {filepath}")
        else:
            self.get_logger().warn("No image available to save.")


def main(args=None):
    rclpy.init(args=args)
    node = ImageViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()