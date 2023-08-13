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

[Simulation of a 4WS Robot Using ROS2 Control and Gazebo - YouTube](https://www.youtube.com/watch?v=VX53gAXafUA&ab_channel=robotmania)

Many nice ROScon workshops

[Resources &mdash; ROS2_Control: Rolling Aug 2023 documentation](https://control.ros.org/master/doc/resources/resources.html#ros-world-2021)



### Installation related notes

- sudo apt update && sudo apt install -y   build-essential   cmake   git   python3-colcon-common-extensions   python3-pip   python3-rosdep   python3-vcstool   wget

- rosdep update **--include-eol-distros**  This is important for foxy and it has to be done in /src.

- rosdep install -y -i --from-paths . ----> again in /src


#### These works
ros2 launch ros2_control_demo_example_5 rrbot_system_with_external_sensor.launch.py
ros2 launch gazebo_ros2_control_demos cart_example_position.launch.py


-- load controllers in different ways:
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'diff_drive_base_controller'],
        output='screen'
    )

    or

        joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
    )
    while it is defined in config.yaml
        joint_broad:
      type: joint_state_broadcaster/JointStateBroadcaster

      or simply using terminal

## Sensors