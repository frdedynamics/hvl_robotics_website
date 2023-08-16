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

### Test and tweak your script
Now comes the fun part! 

Make sure the turtlebot is in a same environment before you start controlling it!
{: .notice--danger}  

It's time to run your simple navigation script, this you can do by navigation (in a terminal) in your virtual machine to the folder where the script is located and run it. Since it's a python 3 file, the command line is the following: 
```console
rocotics@ubuntu:~$ python3 FILE_NAME.py
```
Your turtlebot should now follow your simple navigation algorithm! But since this is not the simulated world you have been testing your code in, it's going to be a mess... that's why you have to now tweak the parameters to make it work in this real maze! 

In the same terminal, open your script in Visual Studio Code with this command: 
```console
rocotics@ubuntu:~$ code FILE_NAME.py
```
Don't forget to save the file before you run it again for testing purposes. 

**Hint 1:**
If nothing happends when you launch your script, then you might have some subscriber or publisher problem. 
Double check them and make sure the **qos** is set properly. 
{: .notice--info} 

<!-- from rclpy.qos import qos_profile_sensor_data
self.scan_sub = self.create_subscription(LaserScan, '/scan', self.clbk_laser, qos_profile_sensor_data)-->


<div class="notice--info"">
<h4>Hint 2:</h4>
<p>The parameters you are going to want to tweak are the following:</p>
<ul>
  <li> ::marker lidar angles</li>
  <li> ::marker linear speed</li>
  <li> ::marker anguler speed</li>
  <li > ::marker distance threshold</li>
</ul>
</div>


<!--parameters that worked for me: 
angles: 40 and 320
linear speed 0.1
lidar_threshold: 0.35
angular speed 0.45-->


