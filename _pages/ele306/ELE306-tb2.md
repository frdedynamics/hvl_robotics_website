---
layout: single
title: "ELE306 Turtlebot Lab-2"
permalink: /courses/ele306/tb2
toc: true # The right side bar "on this page"
breadcrumbs: true  # the directory-of-documents type of header navigation
sidebar:
  nav: "ele306"  # the left navigation bar. Choose which category you want.
taxonomy: markup
---

👋 Hello again, Future Roboticists!

Welcome back to the lab sessions! 🚀 
Now that the Turtlebot 3 can make it out of the maze, let's make it a saving robot by helping it pick up objects and recuing them out of the maze! 🤖🔧

<div align="center">
<img src="https://media.giphy.com/media/RQeTBYpyFe1ZhCUTkF/giphy.gif" width="680" height="340" />
</div>

## Equipment
1. Your personal PC with the Virtual Machine running
2. Matlab, the newest version the better
3. A real life Turtlebot 3
4. Good mood!🌈

## Before the lab 
1. Get together in a group of 2-3 people
2. Find your way to Verftet, Øyrane

## Report 
There is no need to hand in a report for this lab. Signed attendance and a **cool video** of the final product will suffice as approved lab exercise. 

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/turtlebot/tb_in_maze_with_cup.png)

## Instructions

### Setting up the Virtual Machine Network
1. Select your virtual machine on the left bar
2. Click Edit virtual machine settings
3. Select Network Adapter
4. Select the first option **Bridged: Connected directly to the physical network** also check the **Replicate physical network connection state**
5. Go to Configure Adapters and ONLY select the wireless adapter which your PC has. In our case it is “Killer(R) Wi-Fi 6 AX1650 160MHz Wireless Network Adapter”
6. Save everything and start your virtual machine.

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/vm/VM-settings.png)

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/vm/vm_bridge_settings.png)

### Connecting to the Turtlebot
Once your Virtual Machine is on, make sure the networking setting worked, by checking that you are connectec to the World Wide Web. 
This you can do by checking the nettwork status in the upper right corner (should show an ethernet wired connection) or by just opening the browser. 

It's time to turn on the Turtlebot 3 you've been assigned! Plug the battery in and turn it own by using the little switch under the camera. The robot should make a little sound as tit turns on. 

While the turtlebot get's ready, open a terminal in the Virtual Machine: `Crtl + Alt + t `
```console
rocotics@ubuntu:~$ ifconfig
```
make sure that the IP address shown is in the same local network as the Turtlebots.

**Info:**
Being in the same local network can be seen by having the same three first numbers of the IP address!
{: .notice--info}  

The IP address of the Turtlebot assigned to you will be provided to you by the teacher. This IP address is what you will use to connect to the Turtlebot. This type of connection is called an **ssh** connection (Secure Socket Shell), it's a network protocole that allows users to access computers on a network in a secure way. 

Open a terminal window `Crtl + Alt + t ` in the virtual machine and use this command by changing IP_ADDRESS_TURTLEBOT with the actual IP address:
```console
rocotics@ubuntu:~$ ssh ubuntu@IP_ADDRESS_TURTLEBOT
```
The terminal should then ask you for a password, the turtlebot's on-board computer's password. Which is **turtlebot**.
Once the password has been accepted, the terminal's prompt will change to the turtlebot's username, like this:
```console
ubuntu@ubuntu:~$ 
```
You are now controlling the turtlebot's on-board computer, a Raspberry Pi 3B. 

### ROS environment configuration
The default middleware that ROS 2 uses for communication is DDS (Data Distribution Service). In DDS, the primary mechanism for having different logical networks share a physical network is known as the Domain ID. ROS 2 nodes on the same domain can freely discover and send messages to each other, while ROS 2 nodes on different domains cannot. All ROS 2 nodes use domain ID 0 by default. To avoid interference between different groups of computers running ROS 2 on the same network, a different domain ID should be set for each group.
So each student will have a domain ID given to you by the teacher. It will be a number between 0 and 101, inclusive.
This domain ID has to be given to the turtlebot and to your virutal machine. 

#### On the turtlebot
So in the **terminal which is connected to the turtlebot**, write those commands:
```console
ubuntu@ubuntu:~$ nano ~/.bashrc
```
This will open a file in the terminal, with the arrows go all the way down until you find: 
**ROS_DOMAIN_ID=30** 

Change the 30 with your personal domain ID, then save and close the file with `Crtl + s ` then `Crtl + x `
You then need to **source** the file you just modified, this is done with this command:
```console
ubuntu@ubuntu:~$ source ~/.bashrc
```

#### On the remote PC
If you also want your virtual machine to be on the same ros2 network that the turtlebot, this is the setup you need to follow. 
For this lab, this will not be necessary. 
{: .notice--info}  

Then the same has to be done in the virtual machine, on what we call the **remote PC**:
```console
rocotics@ubuntu:~$ nano ~/.bashrc
```
This will open a file in the terminal, with the arrows go all the way down until you find: 
**ROS_DOMAIN_ID=30** 

Change the 30 with your personal domain ID, then save and close the file with `Crtl + s ` then `Crtl + x `
You then need to **source** the file you just modified, this is done with this command:
```console
rocotics@ubuntu:~$ source ~/.bashrc
```
The setup is complete! Time to control the turtlebot! 

### Turtlebot bringup
To get all the ROS nodes that will allow you to control the turtlebot up and running, you have to do what we call **bringup**. 
You do that in the **terminal window which is connected to the turtlebot** with this command line: 
```console
ubuntu@ubuntu:~$ ros2 launch turtlebot3_manipulation_bringup hardware.launch.py
```
The turtlebot should make a sound and the open manipulator (the arm) should rise to start position. 

In this lab you will have to control the gripper of the Turtlebot. To send a goal to the gripper, one normally has to implement an action client with a message of type **control_msgs/GripperCommand**. This message type is not available in matlab, it is possible to import a custom ros2 message in matlab following those [instructions](https://www.mathworks.com/help/ros/ug/_mw_6d3d1e8b-6b64-4d0b-95cf-ef6d7a2d3abf.html). Because importing custom messages is very tricky on a Windows computer, we have created an extra ros2 node that will handle the gripper for us. 
This node can be found on the github repo ros2_students/matlab/labs: simple_gripper_client.py. It is a simple ros2 node that subscribes to **/simple_gripper_cmd** which will be published by the matlab script and creates a action client for **/gripper_controller/gripper_cmd** whose action server is running on the Turtlebot (part of the bringup).
You therefore have to make sure that this node is running on your VM before you send gripper commands from Matlab to the Rurtlebot.
**In a new terminal, in the VM, run this file**:
```console
rocotics@ubuntu:~$ python3 simple_gripper_client.py
```

Your turtlebot is now ready to get commands from your script!

### Implementing an arm and gripper controller
Open a new Matlab script and implement a new controller which controls the robotic arm on the Turtlebot and the gripper!
The steps to complete this task will be outlined but the exact code/commands needed you will have to figure out yourself using what you have learned in the lecture, the information available on this website or what you can find on the internet. 🌐

// TO DO
You will control the OpenManipulator (name of the arm) by following those steps: 
- Setup the environment to communicate with the real turtlebot
- Control the arm through a publisher and the gripper through another publisher.
- Define the robotic arm using your knowledge from the lectures and this [documentation](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/specification/)
- Use the inverse kinematics function from the Peter Corke toolbox to define a goal in joint space: it's important to use the joint limits and an initial position to be able to get a solution to the inverse kinematic problem, have a look at this [documentation](https://www.petercorke.com/RTB/r9/html/SerialLink.html)
- Send the goals to the arm
- Send gripper commands to the gripper: have a look at the **simple_gripper_client.py** script to make sure the message is well defined.

Here is a skeleton of the Matlab script that will help control the arm and gripper!

```matlab
%% ELE306 turtlebot lab number 2
clc; 
clear; 
close all;
import ETS3.*

%% Environment setup
% Setting up environment: you have to define YOUR ros domain id


% Initializing ros node, with a name that makes sense

pause(2)
% Creating publisher to /arm_controller/joint_trajector, message type being trajectory_msgs/JointTrajectory

pause(3)


% Defining message for publisher, and joint_names component


%% Defining the robotic arm DH parameters
%% Use the calculations from the arm kinematics lecture.

robot.qlim = [-3.14, +3.14; -1.57, +1.57; -1.40, +1.57; -1.57, 1.57];

% Visualizing the arm on zero position to check that the definition is correct
robot.plot([0, 0, 0, 0]);

%% Control the gripper and make sure it's open
% Create an action client for /gripper_controller/gripper_cmd with action type control_msgs/GripperCommand => this needs to be imported first!


% Make sure the gripper is open  /gripper_controller/gripper_cmd


%% Time for inverse kinematics!

% First position:

% Define a goal in the reference frame of the base of the arm, with orientation!

% Apply inverse kinematics which TAKE INTO ACCOUNT JOINT LIMITS! and an initial position

% Check the result with plot

% Create a publisher message with that goal in joint space


% Do that as many times as needed to get the full trajectory of the arm and then send the message with all positions


%% Gripping an object
% Create a new gripper goal and send it


%% Taking the object up, for safe keeping
% Reproduce what was done before to control the arm and fold it back in a safe way where it can store the cup while it navigates the maze




%%
% Function definitions at the end if needed

```

<div class="notice--info">
<h4>Info 1: this is how you import ros messages to matlab if you want to try or are running Matlab in ubuntu</h4>
<p> Go to the github repo of the messages: https://github.com/ros-controls/control_msgs/tree/foxy-devel. Download the repo from the foxy branch. In the folder where your matlab scripts are, make a new folder called "custom_msgs" and copy the content of control_msgs into that folder. Then in the matlab terminal run this command: ``` folderPath = fullfile(pwd,"custom_msgs"); ``` and then this command: ``` ros2genmsg(folderPath) ```. It will take a bit of time, but once it's done you should see the **control_msgs/GripperCommand** when running ``` ros2 msg list ``` </p>
</div>

### Fuse your scripts
Now comes the fun part! :)

It's time to fuse the part where the turtlebot picks up the cup, the navigation out of the maze from last lab and the part where the turtlebot releases the cup! 

Make sure the turtlebot is in a safe environment before you start controlling it!
{: .notice--danger}  

Start testing and correcting your code and save that poor cup our of this maze! 

