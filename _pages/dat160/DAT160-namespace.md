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

Inside a launch file you can use the following code to launch a node inside a namepsace. **Don’t forget to replace the in uppercase written names.**
```python
def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='PACKAGE_NAME',
            executable='EXECUTABLE_NAME',
            namespace='NAMESPACE_NAME',
            name='NODE_NAME'),
    ])
```

**When referencing topic names inside a Namepspace be aware of the syntax.** Defining the name with a leading “/” means that I will define the full name with all Namespaces e.g. “/tb3_0/odom”. If you don’t put the leading “/” you are defining the name from the Namespace you are in. If I launch my ROS Node inside the Namespace “tb3_0” I can then reference the same topic as before with just “odom”.

## Exercise 1 - Namespaces

Inside the given ROS package **multi_robot_challenge_22** do the following steps:

* Create a python class used for the robot controller
* Create a python class used for the leader
* The robot controller class should subscribe to the LiDAR topic of the Turtlebot in the same Namespace and publish a single value in a topic called "namespace_test"
* Create a launch file which launches the robot controller class 2 times in the same Namespaces as the Turtlebots
* The leader class should subscribe to both “namespace_test” Topics and print out whatever is received. Add an identifier to the print out so that you know which value comes from which robot class.
* In the previously created launch file add a leader class outside of any Namespaces