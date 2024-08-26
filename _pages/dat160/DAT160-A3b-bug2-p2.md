---
layout: single
title: "Bug2"
permalink: /courses/dat160/a3b
toc: true # The right side bar "on this page"
breadcrumbs: true  # the directory-of-documents type of header navigation
sidebar:
  nav: "dat160"  # the left navigation bar. Choose which category you want.
taxonomy: markup
---

In this assignment you will create a controller which uses the Bug2 algorithm for navigating a Turtlebot to a target destination. The goal of this exercise is for you to be able to implement a navigation algorithm and learn to use the different communication patterns available in ROS. In the last assignment you have created a wall-follower and a go-to-point algorithm. In this assignment you will reuse them and connect them to implement the bug2 algorithm. You will also have to use all 3 of the available communciation patterns in ROS2.

## Bug2 Algorithm
The bug2 algorithm requires a known starting and goal position. It will start by computing a line between the start and the goal positions and then basically operates in 2 modes:

1. In default the robot will try to use the go-to-point algorithm to get to the goal as quickly as possible. If the robot encounters an obstacle it will switch into wall-following and remeber at what point it left the intially computed line.
2. It will follow the wall until it hits a point on the line that is closer to the goal then when the robot left the line. At which point the robot changes back into go-to-point.

![alt]({{ site.url }}{{ site.baseurl }}/assets/images/dat160/bug2_p2/bug2_algorithm.jpg)

## Assignment Requirements

* Create a ROS2 package for your custom messages called **bug2_interfaces**
* Create a *service server* in both the wall-follower and the go-to-point controller.
* For the wall-follower service use the **std_srvs/SetBool** message structure.
* For the go-to-point service create a **custom service message** in the previously created bug2_interfaces package with the following structure:
```c
bool move_switch
geometry_msgs/Point target_position
---
bool success
```
* In the bug2 controller script create two *ROS2 service clients* that connect to the servers from wall follower and go-to-point.
* Create a *ROS2 action server* in the bug2 controller script.
* Create a **custom action message** in the bug2_interfaces package with the following structure:
```c
#goal definition
geometry_msgs/Point target_position
---
#result definition
geometry_msgs/Point base_position
---
#feedback definition
geometry_msgs/Point current_position
```
* Create a robot controller script that has an *action client* connected to the bug2 *action server* and sends the navigation goal position to that *action server*.


![alt]({{ site.url }}{{ site.baseurl }}/assets/images/dat160/bug2_p2/bug2_communication.PNG)
