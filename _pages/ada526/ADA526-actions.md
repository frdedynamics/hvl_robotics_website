---
layout: single
title: "Actions"
permalink: /courses/ada526/actions
toc: true # The right side bar "on this page"
breadcrumbs: true  # the directory-of-documents type of header navigation
sidebar:
  nav: "ada526"  # the left navigation bar. Choose which category you want.
taxonomy: markup

my_variable: scripts.html
---

Actions are one of the communication types in ROS 2 and are intended for long running tasks. They consist of three parts: a goal, feedback, and a result.

Actions are built on topics and services. Their functionality is similar to services, except actions are preemptable (you can cancel them while executing). They also provide steady feedback, as opposed to services which return a single response.

By leveraging the strengths of both topics and services, ROS 2 actions are able to set goals, make progress on those goals, and broadcast when they’ve succeeded.

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/Action-SingleActionClient.gif)


Actions use a client-server model, similar to the publisher-subscriber model (described in the topics tutorial). An “action client” node sends a goal to an “action server” node that acknowledges the goal and returns a stream of feedback and a result.

# Understanding how actions work using Turtlesim
Turtlesim actions tutorial [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html).

## Fibonacci sequence with action-client tutorial:
In mathematics, the Fibonacci sequence is a sequence in which each number is the sum of the two preceding ones. Numbers that are part of the Fibonacci sequence are known as Fibonacci numbers: 0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 144, ....

1. Create an action by following the tutorials [here](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Creating-an-Action.html).

2. Continue creating the action server and client to simulate Fibonacci sequence [here](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html).


# References
1. [ros.org](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
2. (Advanced)[design.ros2.org](https://design.ros2.org/articles/actions.html)
