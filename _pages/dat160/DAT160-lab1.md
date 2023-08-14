---
layout: single
title: "DAT160 Lab-1"
permalink: /courses/dat160/tb1
toc: true # The right side bar "on this page"
breadcrumbs: true  # the directory-of-documents type of header navigation
sidebar:
  nav: "dat160"  # the left navigation bar. Choose which category you want.
taxonomy: markup
---



👋 Hey there, Future Roboticists!

Welcome to the lab sessions, where your imagination meets real-world tech! 🚀 Remember that inverse Brainenberg vehicle you rocked in the virtual realm? It's time to see it cruise in real life using the TurtleBot 3 robot.

Our maze is set, your vehicle awaits. Get ready to fine-tune those algorithms and parameters, transforming your virtual success into real-world magic. 🌟

Get set to witness the future unfold, one twisty maze and one parameter tweak at a time. 🤖🔧

<div align="center">
<img src="https://media.giphy.com/media/47EtjlHYFREM5Rznaf/giphy.gif" width="340" height="340" />
</div>



## Equipment
1. Your personal PC with the Virtual Machine running
2. Your code from Assignment 1
3. A real life Turtlebot 3
4. Good mood!🌈

## Before the lab 
1. Make sure you did Assigment 1
2. Get together in a group of 2-3 people
3. Find your way to Verftet, Øyrane

## Report 
There is no need to hand in a report for this lab. Signed attendance and a **cool video** of the final product will suffice as approved lab exercise. 

## Instructions
TO DO!! write this better with pictures and stuff!

- turn on the vm with the bridged network
- turn on the TB
- in the vm go to the baschrc filer and change the 30 to something else
- then ssh into the turtlebot, change said number too 
- then launch the bringup
- then on the vm launcht he python script and have fun changing the parameters to get the tb through the maze , open the python code with "code gile_name.py"

ops: 
from rclpy.qos import qos_profile_sensor_data
self.scan_sub = self.create_subscription(LaserScan, '/scan', self.clbk_laser, qos_profile_sensor_data)

parameters that worked for me: 
angles: 40 and 320
linear speed 0.1
lidar_threshold: 0.35
angular speed 0.45
