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
The implemented routine for detecting the markers and sorting them relies on the behaviour of the already implemented `aruco_node.py`, which locates Aruco AR markers in images and publishes their IDs and poses.

| **Category**        | **Name/Topic**                  | **Type/Description**                                                                          |
|----------------------|---------------------------------|----------------------------------------------------------------------------------------------|
| **Subscriptions**   | `/camera/image_raw`            | `sensor_msgs.msg.Image`                                                                     |
|                      | `/camera/camera_info`         | `sensor_msgs.msg.CameraInfo`                                                                |
| **Published Topics** | `/aruco_poses`                | `geometry_msgs.msg.PoseArray` - Poses of all detected markers (suitable for rviz visualization) |
|                      | `/aruco_markers`              | `ros2_aruco_interfaces.msg.ArucoMarkers` - Array of all poses with corresponding marker ID  |
| **Parameters**       | `marker_size`                 | Size of the markers in meters                                          |
|                      | `aruco_dictionary_id`         | Dictionary used to generate markers                            |
|                      | `image_topic`                 | Image topic to subscribe to (`/camera/image_raw`)                                  |
|                      | `camera_info_topic`           | Camera info topic to subscribe to (`/camera/camera_info`)                         |
|                      | `camera_frame`                | Camera optical frame to use            |



## vision_node.py
This node, integrated with aruco_node.py, is designed to detect a sequence of ArUco markers in a specific order and manage their visualization and publication. It retrieves raw camera images and ArUco marker detections from other topics and processes them to track and display the detected markers.

### Node Functionality
| **Type**          | **Topic**                                | **Message Type**                       | **Description**                                                                 |
|--------------------|------------------------------------------|-----------------------------------------|---------------------------------------------------------------------------------|
| **Subscriptions**  | `/camera/image_raw`                     | `sensor_msgs/Image`                    | Captures the raw image feed from the camera.                                   |
|                    | `/aruco_markers`                        | `ros2_aruco_interfaces/ArucoMarkers`   | Receives the detected ArUco markers, including their IDs and poses, from `aruco_node.py`. |
| **Publications**   | `/aruco_markers/detected_marker_image`   | `sensor_msgs/Image`                    | Publishes an annotated image highlighting the detected ArUco marker to a custom topic for visualization. |
| **Parameters**     | `detected_marker_image_topic`           | -                                       | Specifies the custom topic to publish the detected marker image (default: `aruco_markers/detected_marker_image`). |


### Behavior
At the very beginning the node sets up a publisher for annotated images and subscribers for the raw camera feed and ArUco marker detections;then it initializes a CvBridge for ROS-OpenCV conversions and an image container for processing frames.

After the initialization process, the implemented logic is the following:
- the robot rotates, storing the ID of the seen markers and publishing their image as soon as it sees them for the first time;
- once the markers have been seen at least one time, the list containing the IDs is sorted and the robot looks for them one at a time;
- when the desired marker is found, a frame containing it is published one more time;
- once the sorted list is over, i.e. all the markers' images have been published again in the correct order, the program ends. 
 







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
