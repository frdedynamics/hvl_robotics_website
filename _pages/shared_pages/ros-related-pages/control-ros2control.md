https://www.youtube.com/watch?v=EosEikbZhiM&ab_channel=Roboage

https://www.youtube.com/watch?v=laWn7_cj434&ab_channel=ArticulatedRobotics

Cool graph, redraw.

*****************************Simple joint_state_publisher*****************************

![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/133c7967-5801-4e58-a01d-a9cb7479d6d0/Untitled.png)

*******with ros_control.*******

![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/f317db73-9179-46cd-b06a-f31d7af25e99/Untitled.png)

- Let’s say you created your robot, visualized Rviz, started empty Gazebo, and want to spawn robot manually.
    - `ros2 run gazebo_ros spawn [entity.py](http://entity.py) -topic robot_description -entity my_bot`
    - Change the robot name ************my_bot************.
    
    ## Gazebo ros_control plugins:
    
    Apparently we need this: https://github.com/ros-controls/gazebo_ros2_control/tree/master
    
    Not sure where and how to use it yet.
    

## Control
ROS2 control vs gazebo-plugin

https://control.ros.org/master/doc/gazebo_ros2_control/doc/index.html
https://www.youtube.com/watch?v=4QKsDf1c4hc&ab_channel=ArticulatedRobotics
https://github.com/ros-controls/gazebo_ros2_control

## Sensors