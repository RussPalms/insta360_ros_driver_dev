#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from insta360_ros_driver.tools import rotate_image, split_image, undistort_image, compress_image_to_msg, image_to_msg

class LiveProcessingNode(Node):
    def __init__(self):
        super().__init__('live_processing')

        self.compress_image = True
        self.bridge = CvBridge()
        self.declare_parameter('undistort', False)
        self.declare_parameter('K', [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ])
        self.declare_parameter('D', [0.0, 0.0, 0.0, 0.0])

        self.undistort = self.get_parameter('undistort').value
        K_param = self.get_parameter('K').value
        D_param = self.get_parameter('D').value

        try:
            self.K = np.array(K_param, dtype=np.float64).reshape(3, 3)
            self.get_logger().info(f"K matrix:\n{self.K}")
        except Exception as e:
            self.get_logger().error(f"Error parsing K matrix: {e}")
            self.K = np.eye(3)

        try:
            self.D = np.array(D_param, dtype=np.float64).flatten()
            if self.D.size < 4:
                raise ValueError(f"D vector must have at least 4 elements, but got {self.D.size}")
            self.get_logger().info(f"D vector: {self.D}")
        except Exception as e:
            self.get_logger().error(f"Error parsing D vector: {e}")
            self.D = np.zeros(4)

        self.subscription = self.create_subscription(
            Image,
            '/insta_image_yuv',
            self.processing_callback,
            0
        )

        if self.compress_image:
            self.front_image_pub = self.create_publisher(
                CompressedImage,
                'front_camera/image_raw/compressed',
                0
            )
            self.back_image_pub = self.create_publisher(
                CompressedImage,
                'back_camera/image_raw/compressed',
                0
            )
            if self.undistort:
                self.front_image_undistorted_pub = self.create_publisher(
                    CompressedImage,
                    'front_camera/image/compressed',
                    0
                )

                self.back_image_undistorted_pub = self.create_publisher(
                    CompressedImage,
                    'back_camera/image/compressed',
                    0
                )
        else:
            self.front_image_pub = self.create_publisher(
                Image,
                'front_camera/image_raw',
                0
            )
            self.back_image_pub = self.create_publisher(
                Image,
                'back_camera/image_raw',
                0
            )
            if self.undistort:
                self.front_image_undistorted_pub = self.create_publisher(
                    Image,
                    'front_camera/image',
                    0
                )

                self.back_image_undistorted_pub = self.create_publisher(
                    Image,
                    'back_camera/image',
                    0
                )

    def processing_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            bgr_image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_I420)
            self.get_logger().info(f"Image Size: {bgr_image.shape}")

            back_image, front_image = split_image(bgr_image)

            back_image = rotate_image(back_image, 90)
            front_image = rotate_image(front_image, -90)
            
            if self.compress_image:
                front_image_raw_msg = compress_image_to_msg(front_image, msg.header.stamp)
                back_image_raw_msg = compress_image_to_msg(back_image, msg.header.stamp)

                self.front_image_pub.publish(front_image_raw_msg)
                self.back_image_pub.publish(back_image_raw_msg)

                if self.undistort:
                    front_image_undistorted = undistort_image(front_image, self.K, self.D)
                    back_image_undistorted = undistort_image(back_image, self.K, self.D)
                    front_image_msg = compress_image_to_msg(front_image_undistorted, msg.header.stamp)
                    back_image_msg = compress_image_to_msg(back_image_undistorted, msg.header.stamp)

                    self.front_image_undistorted_pub.publish(front_image_msg)
                    self.back_image_undistorted_pub.publish(back_image_msg)
            else:
                front_image_raw_msg = image_to_msg(front_image, msg.header.stamp)
                back_image_raw_msg = image_to_msg(back_image, msg.header.stamp)

                self.front_image_pub.publish(front_image_raw_msg)
                self.back_image_pub.publish(back_image_raw_msg)

                if self.undistort:
                    front_image_undistorted = undistort_image(front_image, self.K, self.D)
                    back_image_undistorted = undistort_image(back_image, self.K, self.D)
                    front_image_msg = image_to_msg(front_image_undistorted, msg.header.stamp)
                    back_image_msg = image_to_msg(back_image_undistorted, msg.header.stamp)

                    self.front_image_undistorted_pub.publish(front_image_msg)
                    self.back_image_undistorted_pub.publish(back_image_msg)

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridgeError: {e}")
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LiveProcessingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('LiveProcessingNode has been stopped manually.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
