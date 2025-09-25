---
layout: single
title: "ROS Namespaces"
permalink: /courses/dat160/namespace
toc: true # The right side bar "on this page"
breadcrumbs: true  # the directory-of-documents type of header navigation
sidebar:
  nav: "dat160"  # the left navigation bar. Choose which category you want.
taxonomy: markup
---

In ROS, Namespaces are used to allow multiple topics with the same name to exists. It works similar to a folder structure in any operating system where inside one folder no duplicate names are allowed but in a different folder a file can have the exact same name. We therefore use Namespaces to have multiple topics with the same name e. g. when launching multiple Turtlebots we need two different “cmd_vel” topics to control each of the robots.

Inside a launch file you can use the following code to launch a node inside a namespace. **Don’t forget to replace the in uppercase written names.**
```python
import launch
import launch_ros

def generate_launch_description():
    namespace_node = launch_ros.actions.Node(
        package='PACKAGE_NAME',
        executable='EXECUTABLE_NAME',
        namespace='NAMESPACE_NAME',
        name='NODE_NAME'
    )
    
    return launch.LaunchDescription([
        namespace_node,
    ])
```

**When referencing topic names inside a Namespace be aware of the syntax.** Defining the name with a leading “/” means that I will define the full name with all Namespaces e.g. “/tb3_0/odom”. If you don’t put the leading “/” you are defining the name from the Namespace you are in. If I launch my ROS Node inside the Namespace “tb3_0” I can then reference the same topic as before with just “odom”.

## Troubleshooting
Here are a view terminal commands that can be used for troubleshooting:

- **Listing all running ros topics:**
  ```
  ros2 topic list
  ```
- **Inspecting the details of one topic:**
  ```
  ros2 topic info -v TOPIC-NAME
  ```
  **Note:** `TOPIC-NAME` should be replaced with the name of the topic shown in the `ros2 topic list` command.
- **Print out what is published on a topic:**
  ```
  ros2 topic echo TOPIC-NAME
  ```
  **Note:** `TOPIC-NAME` should be replaced with the name of the topic shown in the `ros2 topic list` command.
- **A visual representation of all running ros nodes, their namespace and how they are connected to each other:**
  ```
  ros2 run rqt_graph rqt_graph
  ```


## Exercise
To goal of this exercise is to create a robot_handler node that is launched twice (once in each namespace of the two turtlebots) and a leader that is launched outside any namespace. The robot_handler node subscribes to the lidar topic of the turtlebot in the same namespace and should publish the value at 180 degrees to a topic called `namespace_test`. The leader should subscribe to both `namespace_test` topics and print out what it has received with an indicator from which robot that information comes. **In the python scripts you will find TODO comments that specify exactly what needs to be done.**

Inside the given ROS package **multi_robot_challenge_23** do the following steps:
* In the **ros2_students_25/namespace_exercise** repository you will find the following files: `robot_handler.py` and `leader.py`. Copy them to **ros2_ws/src/multi_robot_challenge_23/multi_robot_challenge_23**.
* Make sure that both `robot_handler.py` and `leader.py` are configured as console scripts in `setup.py` similar to what you have done to set up the `braitenberg_vehicle` assignment.
* Create a launch file which launches the `robot_handler` node two times in the same Namespaces as the two Turtlebots and the `leader` node once outside any namespace. <br/>
  **Note:** you can use the code example in the beginning of this webpage for that.
