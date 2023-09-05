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

👋 Hey there, Future Roboticists!

Welcome to the lab sessions, where your imagination meets real-world tech! 🚀 
Today you are going to implement an inverse Brainenberg vehicle logic on a TurtleBot 3 robot. Your mission, is to help the Turtlebot to make it out of a maze!

Our maze is set, your vehicle awaits. Get ready to program and fine-tune those parameters! 🤖🔧

<div align="center">
<img src="https://media.giphy.com/media/XBpUGMmoGM4DVHoRMZ/giphy.gif" width="680" height="340" />
</div>

## Equipment
1. Your personal PC with the Virtual Machine running
2. Matlab
3. A real life Turtlebot 3
4. Good mood!🌈

## Before the lab 
1. Get together in a group of 2-3 people
2. Find your way to Verftet, Øyrane

## Report 
There is no need to hand in a report for this lab. Signed attendance and a **cool video** of the final product will suffice as approved lab exercise. 

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/turtlebot/tb_in_maze.png)

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
Your turtlebot is now ready to get commands from your script!

### Implementing a simple navigation algorithm 
Open a new Matlab script and implement a controller which navigates your Turtlebot robot through the maze.
The steps to complete this task will be outlined but the exact code/commands needed you will have to figure out yourself using what you have learned in the lecture, the information available on this website or what you can find on the internet. 🌐

You will control the Turtlebot by using two points (+a and -a in degrees) from the 360 degree onboard LiDAR. With the LiDAR data as input, the robot should show the following behaviours:
- No obstacles are detected -> move forward
- Only the left sensor detects an obstacle -> turn right
- Only the right sensor detects an obstacle -> turn left
- Both sensors detect an obstacle -> stop

![alt]({{ site.url }}{{ site.baseurl }}/assets/images/dat160/braitenberg_vehicle/braitenberg_vehicle.png)

Here is a skeleton of the Matlab script that will help your Turtlebot out of this maze!

```matlab
%% ELE306 turtlebot lab number 1
clc; 
clear; 
close all;

% Setting up the environment: you have to define YOUR ros domain id 

% Initializing a ros node

pause(3)
% Creating subscriber to laser scan (you will need those key words "Reliability","besteffort","Durability","volatile","Depth" ) and publisher to cmd velocity

pause(3)

% Defining the message type for the publisher


% Defining variables

% Front left and front right distances
lidar_left_front = 0;
lidar_right_front = 0;

% Front left and front right angles
left_range = ;
right_range = ;

% Distance threshold
lidar_threshold = ;

% For ever loop
while true
    % Reading out the scan data 
    

    % Plotting the scan data for fun :)
    angles = linspace(-pi,pi,360);
    scan = lidarScan(scanData.ranges, angles);
    plot(scan);
    
    % Velocity commands if no obstacle


    % Velocity commands if obstacles on both sides
    if 
       
    else
        % Velocity commands if obstacles on the right side -> turning left
        if 
        
        end 
        % Velocity commands if obstacles on the left side -> turning right
        if 
        
        end
    end 
    % Send velocity commands to turtlebot
   
end
```

### Test and tweak your script
Now comes the fun part! :)

Make sure the turtlebot is in a safe environment before you start controlling it!
{: .notice--danger}  

It's time to run your simple navigation script, start your Matlab script and see what happends ... it's going to be a bit messy ... that's why you have to tweak the parameters to make it work in this maze! 

Change the values of the different parameters and test it on the Turtlebot. Don't forget to save the file before you run it again for testing purposes and don't forget to put your Turtelbot in a safe environment.

<div class="notice--info">
<h4>Hint 1:</h4>
<p>If nothing happends when you launch your script, then you might have some subscriber or publisher problem. 
Double check them and make sure the **qos** is set properly. </p>
</div>

<!-- laserSub = ros2subscriber(braitenberControllerNode,"/scan","sensor_msgs/LaserScan","Reliability","besteffort","Durability","volatile","Depth",5); -->


<div class="notice--info">
<h4>Hint 2:</h4>
<p>The parameters you are going to want to tweak are the following:</p>
<ul>
  <li> lidar angles</li>
  <li> linear speed</li>
  <li> anguler speed</li>
  <li> distance threshold</li>
</ul>
</div>


<!--parameters that worked for me: 
angles: 40 and 320
linear speed 0.1
lidar_threshold: 0.35
angular speed 0.45-->

