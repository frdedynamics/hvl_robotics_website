---
layout: single
title: "Services"
permalink: /courses/ada526/services
toc: true # The right side bar "on this page"
breadcrumbs: true  # the directory-of-documents type of header navigation
sidebar:
  nav: "ada526"  # the left navigation bar. Choose which category you want.
taxonomy: markup

my_variable: scripts.html
---

Services are another method of communication for nodes in the ROS graph. Services are based on a call-and-response model versus the publisher-subscriber model of topics. While topics allow nodes to subscribe to data streams and get continual updates, services only provide data when they are specifically called by a client.

With ROS 2 services, one or many "client" nodes can make requests to a "server" node and wait for its response. These make services great for performing on-demand tasks – like performing on-the-fly computations or simple one-time tasks.


![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/Service-SingleServiceClient.gif)
![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/Service-MultipleServiceClient.gif)




# Example (optional)
Two integer add via services [here](https://www.roboticsunveiled.com/ros2-service-server-and-client-python-and-cpp/). Only the Python section is relevant.


# References
1. [ros.org](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
2. [foxglove.dev](https://foxglove.dev/blog/creating-ros2-services)
