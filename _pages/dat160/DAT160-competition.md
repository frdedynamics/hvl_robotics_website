---
layout: single
title: "Competition"
permalink: /courses/dat160/competition
toc: true # The right side bar "on this page"
breadcrumbs: true  # the directory-of-documents type of header navigation
sidebar:
  nav: "dat160"  # the left navigation bar. Choose which category you want.
taxonomy: markup

my_variable: scripts.html
---

Semester projects on Search & Rescue robot competition

*Here you will find the technical information about your semester project.  For
grading and other details, check Canvas:*

[https://www.hvl.no/studier/studieprogram/emne/2024/dat160](https://www.hvl.no/studier/studieprogram/emne/2023/dat160)

## Introduction

For the group project you will work as part of a team to solve a Search
and Rescue (S&R) scenario with a small team of mobile robots. Typical
skills to be acquired/demonstrated in the project include mobile robot
control, navigation, robot teams, robot software
architectures and Robot Operating System 2 (ROS2).


:-----:|:-----:
![]({{ site.url }}{{ site.baseurl }}/assets/images/dat160/competition/gazebo_world.png) | ![]({{ site.url }}{{ site.baseurl }}/assets/images/dat160/competition/rviz_world.png)


## Search & Rescue with a Robot Team
The task to solve for the project, and the final competition, is a
Search and Rescue (S&R) scenario with a team of 2 mobile robots
(Turtlebots), with the following sensors:

- 1x forward-looking camera
- 1x lidar
- 1x IMU (gyro and accelerometer)
- 2x wheel encoders
  
The high-level goal is to provide assistance to injured persons in the
environment, as well as detect fire sources. The assistance needs to be
communicated by the robots, and is measured through a point system,
which is described in detail below. Completion of the different tasks is done by detecting ArUco markers and reporting their position to the scoring system.

The robots are able to communicate over a network using ROS2 topics,
services or actions. The robots have a limited amount of time to
complete the task, which is set to 10 minutes. There exists 5 variations
of the environment, each with different damage to the infrastructure.
For example, new obstacles, new openings in walls and targets at different
locations. For the competition one of the maps will be chosen at random.

The robots will be given a blueprint map of the environment but not the
locations of the ArUco markers. Thus, the robot controllers need to be able
to handle a range of different situations, and pre-programming paths is
in general not possible or allowed.

:----------:|:------:
![]({{ site.url }}{{ site.baseurl }}/assets/images/dat160/competition/world1.png) | ![]({{ site.url }}{{ site.baseurl }}/assets/images/dat160/competition/world2.png)

:----------:|:------:
![]({{ site.url }}{{ site.baseurl }}/assets/images/dat160/competition/world3.png) | ![]({{ site.url }}{{ site.baseurl }}/assets/images/dat160/competition/world4.png)

:----------:|:------:
![]({{ site.url }}{{ site.baseurl }}/assets/images/dat160/competition/world4.png) | 


In your project report you need to clearly explain how the robots
communicate during S&R, which navigation algorithm is running and how
you implemented those elements. The navigation algorithm needs to be one
where you have done significant implementation yourself, and not an
existing one downloaded as is from the internet.

## Competition rules

### Introduction

You are supposed to develop a search and rescue system using mobile
robots in Gazebo environment. You are given a maze to solve containing 5
Aruco tags representing 3 features:

|Human (ArUco-ID: 2)   | Fire (AruCo-ID: 0, 1, 3) |
|:----------:|:----------:|
| ![]({{ site.url }}{{ site.baseurl }}/assets/images/dat160/competition/human-ar.png) | ![]({{ site.url }}{{ site.baseurl }}/assets/images/dat160/competition/fire-ar.png) |

| Big Fire (AruCo-ID: 4)|
|:------:|
| ![]({{ site.url }}{{ site.baseurl }}/assets/images/dat160/competition/big-fire-ar.png)|
  

Your aim is to detect all the Aruco tags in the environment, inform
about their location and distinguish if it is a fire or a human. You
have 10 minutes (600 secs) in total to finish the task. Each
successfully detected Aruco tag will give you points.

You can spawn as many robots as you want. But all robots have to start in the starting zone. Big fire requires at least 2 robots to meet at the fire location to extinguish. Therefore, robots should communicate with each other.
  

### Point system

Points are allocated according to the following rules:

- id: 0,1,3 are small fires (+100pt)
- id: 2 is the human (+100pt)
- id: 4 is the big fire. For this tag both robots should meet up
  (+100pt finding, +300pt if two robots meet there, i.e. if at any
  point after reporting id:4 the two robots are within a 2 meter radius of this Aruco tag location)

### Reporting targets

To receive points your robots have to report their findings. This is done using a ROS service communication structure. To run the scoring system you can either run the following lines in a terminal:
```bash
ros2 run scoring scoring
```
or add the following lines to a launch file:
```python
launch_ros.actions.Node(
  package='scoring',
  executable='scoring',
  name='scoring'),
```

The scoring system runs a service server which uses the SetMarkerPosition type which is from the scoring_interfaces package and is defined as:
```
int8 marker_id
geometry_msgs/Point marker_position
---
bool accepted
```


### Final competition

The final competition will run on a dedicated computer provided by the organizers. In the last week before the competition you will have the chance to try your project on the competition PC. For this we will require you to make an appointment and provide us with your git repository. Make sure that your repository also contains instruction on how to run your project. The environment is randomly chosen by the organizers from the given map pool. The score will be maintained by a dedicated score-counting node controlled by the organizers. The winner of the competition will be the group that has got the best score in the chosen environment. Note that we will also be watching during the competition, to ensure that the score is actually correct in case of bugs.
