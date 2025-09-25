---
layout: single
title: "ArUco Markers"
permalink: /courses/dat160/aruco-markers
toc: true # The right side bar "on this page"
breadcrumbs: true  # the directory-of-documents type of header navigation
sidebar:
  nav: "dat160"  # the left navigation bar. Choose which category you want.
taxonomy: markup
---

![alt]({{ site.url }}{{ site.baseurl }}/assets/images/dat160/aruco_markers/aruco_marker.png)

ArUco markers are a type of fiducial marker used in augmented reality and computer vision applications. They are square-shaped patterns consisting of a black-and-white, binary-coded grid that can be easily detected and decoded by computer algorithms. ArUco markers are designed to be robust and easily recognizable by cameras, making them useful for tracking and positioning purposes.

Here are key aspects of ArUco markers:
1. **Design**: Each marker has a unique pattern that can be identified and used to convey specific information. The black-and-white contrast makes them easy to detect under varying lighting conditions.
2. **Identification**: ArUco markers contain a unique ID that can be decoded by software, allowing systems to recognize different markers and their orientations.
3. **Detection**: They can be detected in real-time using computer vision algorithms that analyze video streams or images, often leveraging libraries like OpenCV and tools specifically designed for handling ArUco markers.
4. **Pose Estimation**: Besides identification, ArUco markers can be used for estimating the pose of a camera relative to the marker. This involves determining the position and orientation of the camera in 3D space, which is crucial for applications such as augmented reality.
5. **Applications**: Common use cases include robotics (for navigation and positioning), augmented reality (to align virtual objects with real-world environments), and camera calibration.

ArUco markers are part of the OpenCV library, particularly in the `cv2.aruco` module, which provides tools for creating and detecting these markers effectively. They are advantageous due to their ease of use, straightforward detection algorithms, and the ability to be generated programmatically for various sizes and IDs.

## ArUco Markers in the Semester Project
When using the existing ArUco marker detection there are a view things that are important:
- Make sure your have the file `marker_pose.py` in your semester project (example path: **ros2_ws/src/multi_robot_challenge_23/multi_robot_challenge_23**)
- While any of the maps of the semester project are running, use the `ros2 topic list` command in another terminal window. You should be able find the following topics:
  - `/NAMESPACE/aruco_markers`
  - `/NAMESPACE/aruco_poses`
  - `/NAMESPACE/marker_id`
  - `/NAMESPACE/marker_map_pose`<br/>
  **Note:** `NAMESPACE` would be replaced with either `tb3_0` or `tb3_1`, e.g. `/tb3_0/marker_map_pose`.

The robot will publish the id of the detected ArUco marker on the `marker_id` topic. The `marker_map_pose` topic is used to publish the position and orientation of the detected marker in the map coordinate system.<br/>
**DON'T USE EITHER `aruco_markers` or `aruco_poses` TOPICS AS THESE WILL GIVE YOU THE POSITION AND ORIENTATION OF THE MARKER IN THE CAMERA COORDINATE SYSTEM INSTEAD OF THE MAP COORDINATE SYSTEM!**



## Exercise
In this exercise you will finish the marker_detection script, which subscribes to the ArUco marker detection topics on one of the two turtlebots in your semester project and whenever a new marker is detected it will print out the ID and pose (position and orientation) of that marker.

Inside the given ROS package **multi_robot_challenge_23** do the following steps:
* In the **ros2_students_25/aruco_marker_exercise** repository you will find the following files: `marker_detection.py` and `marker_pose.py`. Copy them to **ros2_ws/src/multi_robot_challenge_23/multi_robot_challenge_23**.
  **Note:** you might already have `marker_pose.py` in your semester project. In that case only copy `marker_detection.py`.
* Make sure that both `marker_detection.py` and `marker_pose.py` are configured as console scripts in `setup.py` similar to what you have done to set up the `braitenberg_vehicle` assignment.
* Complete the exercise by following the **TODOs** in the `marker_detection.py` script.
* To test your solution do the following steps:
  - Launch one of the semester project maps. E.g. `ros2 launch multi_robot_challenge_23 rescue_robots_w1.launch.py`.
  - Run the marker detection script. E.g. `ros2 run multi_robot_challenge_23 marker_detection`.
  - Place the turtlebot manually in gazebo in front of one of the ArUco markers. **Make sure the robot is far enough away from the marker that the camera can see the whole marker.**
  - You should get an output in the terminal of your marker detection script that tells you the ID and pose of the marker.


