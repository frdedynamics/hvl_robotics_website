In this tutorial, you will learn about simulation in ROS using Gazebo. You will also learn briefly the physical andy dynamic (inertia and collision) properties of an object and how to create a simulated world.

# Gazebo

Gazebo is the most used simulator in ROS. It has integrated physics rules like gravity, colision, light distribution with reflection and refraction properties, mass and inertia etc. Now, we will see how to set the Gazebo for very basic usage.

{: .notice--info}
You should always remember. Although RViz and Gazebo seems pretty similar, they are used in completely different purposes. Rviz is for visualization (of robots, sensors etc.), Gazebo is for simulation (of robots, objects, environment etc.).

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/mybot.png)

## Create a launch file for Gazebo