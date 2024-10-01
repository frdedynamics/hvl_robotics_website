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

Referred tutorial notes: 
https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html

https://design.ros2.org/articles/actions.html

https://docs.ros.org/en/foxy/Tutorials/Intermediate/Creating-an-Action.html

# Actions

Actions are one of the communication types in ROS 2 and are intended for long running tasks. They consist of three parts: a goal, feedback, and a result.

Actions are built on topics and services. Their functionality is similar to services, except actions are preemptable (you can cancel them while executing). They also provide steady feedback, as opposed to services which return a single response.

By leveraging the strengths of both topics and services, ROS 2 actions are able to set goals, make progress on those goals, and broadcast when they’ve succeeded.

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/Action-SingleActionClient.gif)


Actions use a client-server model, similar to the publisher-subscriber model (described in the topics tutorial). An “action client” node sends a goal to an “action server” node that acknowledges the goal and returns a stream of feedback and a result.

## Example coundown action server:
https://foxglove.dev/blog/creating-ros2-actions
