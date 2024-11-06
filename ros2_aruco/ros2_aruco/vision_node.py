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

# Camera params
height = 480
width = 640

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.get_logger().info("Vision Node Initialized.")

        # Create custom topic to publish the detected marker
        self.declare_parameter("detected_marker_image_topic", "aruco_markers/detected_marker_image")
        self.detected_marker_image_pub = self.create_publisher(Image, self.get_parameter("detected_marker_image_topic").value, qos_profile=qos_profile_sensor_data)
        self.get_logger().info("Created publisher to /aruco_markers/detected_marker_image")

        # Create subscriber to aruco_markers topic
        self.id_retriever = self.create_subscription(ArucoMarkers, 'aruco_markers', self.aruco_callback, qos_profile=qos_profile_sensor_data)     


        # Create a subscriber for the Image topic with queue size of 1
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.listener_callback, 1)
        self.get_logger().info('Subscribing to /camera/image_raw topic')

        # Initialize the CvBridge
        self.bridge = CvBridge()
        self.get_logger().info('CvBridge initialized')

        # Initialize an image container (to store the received frame)
        self.image = np.zeros((height, width, 3), dtype=np.uint8)  # Initialize with a black image
        self.get_logger().info('Image container initialized')


    def listener_callback(self, msg):

        # Convert image from ROS msg on topic /camera_image/image_raw to OpenCV image
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.get_logger().info(f"Received an image of size: {self.image.shape}")
        #self.get_logger().info('Image converted to OpenCV format')   


    def aruco_callback(self, msg):

        # Get the ArUco marker id
        detected_marker_id = msg.marker_ids
        print(f"Received ArUco markers: {list(detected_marker_id)}")


        # Get the camera info for using cv2.circle
        color = (0, 255, 0) # Green
        thickness = 2
        radius = 20

        # Camera params
        # height = 480
        # width = 640


        # Draw a circle around the detected ArUco marker
        for i in range(len(detected_marker_id)):

            # Get the detected ArUco marker id
            marker_id = detected_marker_id[i]

            # check if marker_id is valid
            if marker_id > 40:
                break

            current_frame = self.image

            # Get the detected ArUco marker pose
            marker_pose = msg.poses[i] 

            # Draw a circle around the detected ArUco marker
            # Get the center of the detected ArUco marker
            marker_position_point = (marker_pose.position.x, marker_pose.position.y, marker_pose.position.z)

            # Convert the marker position from camera frame to image frame
            center = (int(marker_position_point[0] * width), int(marker_position_point[1] * height))

            # Draw a circle around the detected ArUco marker
            cv2.circle(current_frame, center, radius, color, thickness)

            # Publish the image with the detected ArUco marker
            self.detected_marker_image_pub.publish(self.bridge.cv2_to_imgmsg(current_frame, encoding="bgr8"))
            self.get_logger().info(f"Published an image of size: {current_frame.shape}")

            # Display the result image
            # Create a window
            cv2.namedWindow('Detected ArUco Marker')
            cv2.imshow('Detected ArUco Marker', current_frame)
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = VisionNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
