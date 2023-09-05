---
layout: single
title: "DAT160 Lab-2"
permalink: /courses/dat160/tb2
toc: true # The right side bar "on this page"
breadcrumbs: true  # the directory-of-documents type of header navigation
sidebar:
  nav: "dat160"  # the left navigation bar. Choose which category you want.
taxonomy: markup
---



👋 Heello again, Future Roboticists!

Welcome back to the lab sessions! 🚀 In the last lab, you implemented in real life an inverse Brainenberg vehicle on a Turtlebot 3 robot.

But it could not handle corners, so we had to make it easier for the Turtlebot by covering up the corners. 

The time for simple navigation is over! It's time to uncover the maze's corners and implement a better solution to our maze. 🌟

It's your turn to use the assigments 2 and 3 and create a wall following algorithm for this maze and robot and you will again need to tweak the different parameters! 🤖🔧

<div align="center">
<img src="https://media.giphy.com/media/3o6ZthgwwYimHPVTaM/giphy.gif" width="340" height="340" />
</div>



## Equipment
1. Your personal PC with the Virtual Machine running
2. Your code from Assignments 2 and 3
3. A real life Turtlebot 3
4. Good mood!🌈

## Before the lab 
1. Make sure you did Assigment 2 and 3
2. Get together in a group of 2-3 people
3. Find your way to Verftet, Øyrane

## Report 
There is no need to hand in a report for this lab. Signed attendance and a **cool video** of the final product will suffice as approved lab exercise. 

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/turtlebot/tb_in_maze_2.png)

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


### Implement a wall follower algorithm
Let's hack together some wall follower algorithm! 

As for your assigments, it is expected of you that you write a python script. For this lab it is anough with the python file, no need to make a ros2 package. 
This project is very close to your assigments so re-use them as much as possible!

<div class="notice--warning">
<h3>Requirements: </h3>
<h4>The wall follower should find a wall and then drive in parallel to it, using the LiDAR sensor topic (/scan) to ensure the robot is not straying off and to reorient itself when hitting a corner.
A special case you will also need to take care off is when the wall suddenly stops you need to program a behavior which moves the robot to the other side of the wall so that it can continue with the normal wall following.
For moving the robot create a publisher for the /cmd_vel topic. Inside the control loop use a state variable to define which "mode" the navigation is in and switch between different control "modes".
Don't hesitate to measure the maze and make small schematics to help you with the logic. </h4>
</div>

### Test and tweak your script
Now comes the fun part! 

Make sure the turtlebot is in a safe environment before you start controlling it!
{: .notice--danger}  

It's time to run your wall follower script, this you can do by navigation (in a terminal) in your virtual machine to the folder where the script is located and run it. Since it's a python 3 file, the command line is the following: 
```console
rocotics@ubuntu:~$ python3 FILE_NAME.py
```
Your turtlebot should now follow your wall follower algorithm! But it's going to be a mess... that's why you have to now tweak the parameters to make it work in this real maze! 

In the same terminal, open your script in Visual Studio Code with this command: 
```console
rocotics@ubuntu:~$ code FILE_NAME.py
```
Don't forget to save the file before you run it again for testing purposes. 

<div class="notice--info">
<h4>Hint 1:</h4>
<p>If nothing happends when you launch your script, then you might have some subscriber or publisher problem. 
Double check them and make sure the **qos** is set properly. </p>
</div>

<!-- from rclpy.qos import qos_profile_sensor_data
self.scan_sub = self.create_subscription(LaserScan, '/scan', self.clbk_laser, qos_profile_sensor_data) -->


<div class="notice--info">
<h4>Hint 2:</h4>
<p>The parameters you are going to want to tweak are the following:</p>
<ul>
  <li> lidar angles: front, front right, front left, left, right, back right, back left</li>
  <li> linear speeds for each mode or state</li>
  <li> anguler speeds for each mode or state</li>
  <li> distance thresholds front, sides, etc</li>
</ul>
</div>


<!--parameters that worked for me: 
'right':  msg.ranges[269],
'fright': msg.ranges[314],
'bright': msg.ranges[224],
'front':  msg.ranges[0],
'fleft':  msg.ranges[44],
'left':   msg.ranges[89],

 d = 0.25
 d_45 = d/math.cos(math.pi/4)
 d_error = d/10
 d_front = d + 0.15

  def find_wall(self):
      msg = Twist()
      msg.linear.x = 0.1
      msg.angular.z = -0.5
      return msg
  
  def turn_left(self):
      msg = Twist()
      msg.angular.z = 0.2
      return msg
  
  def follow_the_wall(self):
      msg = Twist()
      msg.linear.x = 0.1
      return msg  -->


