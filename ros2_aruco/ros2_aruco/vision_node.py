import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
from ros2_aruco import transformations

from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.get_logger().info("Vision Node Initialized.")

        # Create custom topic to publish the detected marker
        self.declare_parameter("detected_marker_image_topic", "/detected_marker_image")
        


        # Create subscriber to aruco_markers topic
        self.id_retriever = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.aruco_callback,
            qos_profile=qos_profile_sensor_data
        )


    def aruco_callback(self, msg):

        # Get the ArUco marker id
        detected_marker_id = msg.marker_ids
        print(f"Received ArUco markers: {list(detected_marker_id)}")



def main():
    rclpy.init()
    node = VisionNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
