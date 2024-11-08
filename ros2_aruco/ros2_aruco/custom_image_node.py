import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import cv_bridge
import numpy as np

# Camera params
height = 480
width = 640

class ImageViewer(Node):
    def __init__(self):
        super().__init__('image_viewer')
        self.subscription = self.create_subscription(
            Image,
            '/aruco_markers/detected_marker_image',
            self.listener_callback,
            1)
        self.bridge = cv_bridge.CvBridge()

        # Initialize an image container (to store the received frame)
        self.image = np.zeros((height, width, 3), dtype=np.uint8) # Initialize with a black image
        self.get_logger().info('Image container initialized')

    def listener_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imshow("Image Window", self.image)
            self.get_logger().info('Received an image')
        except CvBridgeError as e:
            print(e)

def main(args=None):
    rclpy.init(args=args)
    image_viewer = ImageViewer()
    rclpy.spin(image_viewer)
    image_viewer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
