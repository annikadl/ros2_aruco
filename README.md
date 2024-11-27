# Ros2_aruco

This repository contains the Vision Package for the first assignment of the Experimental Robotics Laboratory Course at the Robotics Engineering Master Course of Genoa.
The authors are:
- Valentina Condorelli, ;
- Annika Delucchi, 49759849;
- Ramona Ferrari, ;
- Daniele Rialdi, .

The package was initially forked by the Professor's repository, to which we added the file `vision_node.py` at the path `ros2_aruco/ros2_aruco/`.

## Assignment
The assignment required to:
- spawn the robot in Gazebo, surrounded by 5 Aruco markers arranged in a circle;
- implement a routine that starts finding the marker with the lowest ID, and then finds all markers in order;
- each time a marker is found, a new image is published on a custom topic with a circle around the marker found;
- implement the same behaviour with two different nodes: in the first one, the whole robot moves; in the second one, you find the markers by only moving the camera.

Note that to accomplish these tasks, the behaviour of this `vision_node` must be coupled with the control routine that can be found at this link "".

## Already existing nodes
The implemented routine for detecting the markers and sorting them relies on the behaviour of the already implemented `aruco_node.py`, that works as follows:



## vision_node.py
This node, integrated with aruco_node.py, is designed to detect a sequence of ArUco markers in a specific order and manage their visualization and publication. It retrieves raw camera images and ArUco marker detections from other topics and processes them to track and display the detected markers.

### Node Functionality
| **Type**          | **Topic**                                | **Message Type**                       | **Description**                                                                 |
|--------------------|------------------------------------------|-----------------------------------------|---------------------------------------------------------------------------------|
| **Subscriptions**  | `/camera/image_raw`                     | `sensor_msgs/Image`                    | Captures the raw image feed from the camera.                                   |
|                    | `/aruco_markers`                        | `ros2_aruco_interfaces/ArucoMarkers`   | Receives the detected ArUco markers, including their IDs and poses, from `aruco_node.py`. |
| **Publications**   | `/aruco_markers/detected_marker_image`   | `sensor_msgs/Image`                    | Publishes an annotated image highlighting the detected ArUco marker to a custom topic for visualization. |
| **Parameters**     | `detected_marker_image_topic`           | -                                       | Specifies the custom topic to publish the detected marker image (default: `aruco_markers/detected_marker_image`). |



Key Behavior
Initialization:

Sets up a publisher for annotated images and subscribers for the raw camera feed and ArUco marker detections.
Initializes a CvBridge for ROS-OpenCV conversions and an image container for processing frames.
Processing Logic:

Image Callback: Updates the internal image container with the latest camera frame.
Aruco Callback:
Projects 3D marker positions onto the 2D image plane using the camera’s intrinsic parameters.
Highlights detected markers on the current frame with green circles.
Tracks the IDs of markers already seen and ensures each is detected and published once.
After all markers are detected, reorders and processes them sequentially by ID for further display or action.
Tracking and Visualization:

Initially tracks up to five markers, ensuring each is detected once in any order.
Once all markers are detected, it processes them in ascending order of their IDs for structured visualization.
Displays the processed images in a window and publishes them to the specified topic.
Application
This node is ideal for tasks requiring structured recognition and ordered processing of ArUco markers, such as:

Sequential marker-based navigation.
Visual tracking and marker-specific actions.
Marker identification and logging for robotics applications.
Authors
Valentina Condorelli, Annika Delucchi, Ramona Ferrari, Daniele Rialdi

  







# old readme
ROS2 Wrapper for OpenCV Aruco Marker Tracking

This package depends on a recent version of OpenCV python bindings:

```
pip install opencv-contrib-python # or pip3
```

## ROS2 API for the ros2_aruco Node

This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
* `/camera/image_raw` (`sensor_msgs.msg.Image`)
* `/camera/camera_info` (`sensor_msgs.msg.CameraInfo`)

Published Topics:
* `/aruco_poses` (`geometry_msgs.msg.PoseArray`) - Poses of all detected markers (suitable for rviz visualization)
* `/aruco_markers` (`ros2_aruco_interfaces.msg.ArucoMarkers`) - Provides an array of all poses along with the corresponding marker ids

Parameters:
* `marker_size` - size of the markers in meters (default .0625)
* `aruco_dictionary_id` - dictionary that was used to generate markers (default `DICT_5X5_250`)
* `image_topic` - image topic to subscribe to (default `/camera/image_raw`)
* `camera_info_topic` - Camera info topic to subscribe to (default `/camera/camera_info`)
* `camera_frame` - Camera optical frame to use (default to the frame id provided by the camera info message.)

## Generating Marker Images

```
ros2 run ros2_aruco aruco_generate_marker
```

Pass the `-h` flag for usage information: 

```
usage: aruco_generate_marker [-h] [--id ID] [--size SIZE] [--dictionary]

Generate a .png image of a specified maker.

optional arguments:
  -h, --help     show this help message and exit
  --id ID        Marker id to generate (default: 1)
  --size SIZE    Side length in pixels (default: 200)
  --dictionary   Dictionary to use. Valid options include: DICT_4X4_100,
                 DICT_4X4_1000, DICT_4X4_250, DICT_4X4_50, DICT_5X5_100,
                 DICT_5X5_1000, DICT_5X5_250, DICT_5X5_50, DICT_6X6_100,
                 DICT_6X6_1000, DICT_6X6_250, DICT_6X6_50, DICT_7X7_100,
                 DICT_7X7_1000, DICT_7X7_250, DICT_7X7_50, DICT_ARUCO_ORIGINAL
                 (default: DICT_5X5_250)
```
