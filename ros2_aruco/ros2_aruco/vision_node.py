import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers

# Camera params
height = 480
width = 640

seen_ids = []

# Intrinsic camera matrix: used for mapping 3D points to 2D image plane
camera_matrix = np.array([
    [355.39477962339123, 0.0, 320.5],
    [0.0, 355.39477962339123, 240.5],
    [0.0, 0.0, 1.0]
])

# Distortion coefficients (all zeros : check ros2 topic echo /camera/camera_info)
dist_coeffs = np.zeros(5)

# Variable to save the id of the last seen marker
global last_id
last_id = 0

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.get_logger().info("Vision Node Initialized.")

        # Create custom topic to publish the detected marker
        self.declare_parameter("detected_marker_image_topic", "aruco_markers/detected_marker_image")
        self.detected_marker_image_pub = self.create_publisher(Image, self.get_parameter("detected_marker_image_topic").value, qos_profile=qos_profile_sensor_data)
        self.get_logger().info("Created publisher to /aruco_markers/detected_marker_image")
        cv2.namedWindow('Detected ArUco Marker')

        # Subscriber to ArucoMarkers topic
        self.id_retriever = self.create_subscription(ArucoMarkers, 'aruco_markers', self.aruco_callback, qos_profile=qos_profile_sensor_data)

        # Subscriber for the Image topic
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.listener_callback, 1)
        self.get_logger().info('Subscribing to /camera/image_raw topic')

        # Initialize CvBridge
        self.bridge = CvBridge()
        self.get_logger().info('CvBridge initialized')

        # Initialize an image container (to store the received frame)
        self.image = np.zeros((height, width, 3), dtype=np.uint8) # Initialize with a black image
        self.get_logger().info('Image container initialized')

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        #self.get_logger().info(f"Received an image of size: {self.image.shape}")
        #self.get_logger().info('Image converted to OpenCV format') 
        
    def aruco_callback(self, msg):
    	
        # Get detected marker IDs and poses
        detected_marker_ids = msg.marker_ids
        # Get detected marker poses
        detected_marker_poses = msg.poses

        color = (0, 255, 0)  # Green
        thickness = 2
        radius = 20

        current_frame = self.image.copy()

        for i, marker_id in enumerate(detected_marker_ids):
            if marker_id > 40:
                continue

            # Extract the pose of the detected marker
            marker_pose = detected_marker_poses[i]
            marker_position_point = np.array([[marker_pose.position.x, marker_pose.position.y, marker_pose.position.z]])

            # Project 3D points to the 2D image plane
            projected_points, _ = cv2.projectPoints(marker_position_point, (0, 0, 0), (0, 0, 0), camera_matrix, dist_coeffs)
            center = tuple(projected_points[0][0].astype(int))

            # Draw the circle at the projected 2D position
            cv2.circle(current_frame, center, radius, color, thickness)

            # Publish the image with the detected ArUco marker
            # check if the image with this marker has already been published
            if marker_id not in seen_ids:
                self.detected_marker_image_pub.publish(self.bridge.cv2_to_imgmsg(current_frame, encoding="bgr8"))
                self.get_logger().info(f"Published an image with detected marker at: {center} with id {marker_id}")
                seen_ids.append(marker_id)
                # Display the result image

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
