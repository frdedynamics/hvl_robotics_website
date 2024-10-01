---
layout: single
title: "Qt libraries with ROS"
permalink: /courses/ada526/rqt
toc: true # The right side bar "on this page"
breadcrumbs: true  # the directory-of-documents type of header navigation
sidebar:
  nav: "ada526"  # the left navigation bar. Choose which category you want.
taxonomy: markup

my_variable: scripts.html
---
## Installation
This is a tutorial for installation of Qt 5.7.0 to Ubuntu 12.10. It may be used for some newer versions of Qt and Ubuntu.

Install Qt 5: 
sudo apt install build-essential qtcreator qt5-default qtbase5-examples

Install Qt to Python converter:
sudo apt install pyqt5-dev-tools


If error: g++: Command not found
sudo apt-get purge build-essential
sudo apt-get install build-essential

If error: GL/gl.h: No such file or directory
sudo apt-get install mesa-common-dev

## Tutorial start
In this tutorial, we will create a GUI package and interact with ROS nodes and messages via this package.

### Create a package
You can jump to the next section if you don't want to use Qt with ROS. Otherwise, you need to have your files in your ros2_ws. Therefore, we start by creating a new ROS package for this tutorial.

cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python --node-name my_gui_pkg my_gui_pkg
cd ..
colcon build
cd ~/ros2_ws/src/m
mkdir ui

### Design your widget
Open Qt Designer
Select Widget and default screen size
Press create
Select two "Push Button"s and one "Plain Text Edit", and place them somewhere in the Widget window.
Change their name something nice, and give a representative class names.

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/qt-push-button.png)

Save by hitting Ctrl+s
Select the "ui" folder in your ROS package.
Change the file name to "main.ui"

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/qt-main-ui.png)


### Connecting UI and ROS
There are four steps:

1. Converting UI to Python script.
2. We will create a dummy publisher for test purposes and an actual node to run our GUI.
3. We will start this publisher with "Run ROS node" button.
4. We will set a value a for an action wit "Set action" button by reading from the plain text edit.




#### Converting UI to Python script

Go to the direction where your "main.ui" is:
~/ros2_ws/src/my_gui_pkg/ui

Convert the UI file into a Python file:
pyuic5 -x main.ui -o main.py

Note that whenever you change something in your design, you must convert it into the Python file.

Open VS code and find your package in the explorer.
Add this line at the top: #!/usr/bin/env python3


#### Creating necessary nodes:
TODO
we have my_gui_pkg already. We will

add the ui folder path
Add as a class -- never change the generated python code manually, otherwise you will lose everything you have done when you edit your UI via Qt Designer.






