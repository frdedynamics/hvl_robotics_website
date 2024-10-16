---
layout: single
title: "Machine Learning"
permalink: /courses/dat160/a4
toc: true # The right side bar "on this page"
breadcrumbs: true  # the directory-of-documents type of header navigation
sidebar:
  nav: "dat160"  # the left navigation bar. Choose which category you want.
taxonomy: markup
---

In this assignment you will work with the code example with KerasCV in Google Collab found here:

https://colab.research.google.com/github/keras-team/keras-io/blob/master/guides/ipynb/keras_cv/object_detection_keras_cv.ipynb

You will explore how well this pretrained YOLO network can detect images of cats and dogs on the walls of a virtual environment in Gazebo, from the point of view of the Turtlebot robot. You will work with a teleoperated Turtlebot in a simulated environment. The robot is equipped with a camera that provides a stream of images to a ```'/camera/image_raw'``` topic. 

Your task is TODO.

![alt]({{ site.url }}{{ site.baseurl }}/assets/images/dat160/robot_vision/rv_1.gif)
_TODO._

## Setup Process
* Open a new terminal by pressing: Ctrl+Alt+T
* Change directory into the source folder of your ROS2 workspace:
```bash
cd ~/ros2_ws/src
```
* Create a ROS2 python package with the name robot_ml:
```bash
ros2 pkg create --build-type ament_python robot_ml
```
* Copy the folders **launch**, **models** and **worlds** from the **ros2_students_24/robot_ml** git repository to your newly created package (ros2_ws/src/robot_ml)
* In ROS2 the python scripts of packages are located in a folder with the same name as the package. Copy the files **camera_viz.py** and **project_ml_idea.py** from the **ros2_students_24/robot_ml** git repository to the scripts folder of your package (ros2_ws/src/robot_ml/robot_ml)
- In the **setup.py** file of your package add the following lines:
TODO
