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
```python
# Add after: from setuptools import setup
import os
from glob import glob
# In order to be able to access the added files during runtime we add we add them to data_files
# Add after: ('share/' + package_name, ['package.xml']),
(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
(os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
# To make our python script excecutable with ros2 run, we add a entry_points definition
# Add after: 'console_scripts': [
  'camera_viz = robot_ml.camera_viz:main',
  'project_ml_idea = robot_ml.project_ml_idea:main',
```
* Move to workspace directory, build your workspace and source it:
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
* To launch the Gazebo simulation with the Turtlebot in an environment with images of cats and dogs on the walls use:
```bash
ros2 launch robot_vision spawn_robot.launch.py
```
