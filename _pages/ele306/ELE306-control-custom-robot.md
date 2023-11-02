---
layout: single
title: "Control a Custom Robot Simulation"
permalink: /courses/ele306/control-custom-robot
toc: true # The right side bar "on this page"
breadcrumbs: true  # the directory-of-documents type of header navigation
sidebar:
  nav: "ele306"  # the left navigation bar. Choose which category you want.
taxonomy: markup
# {% include_absolute '_pages/shared_pages/ros-related-pages/build-custom-robot.md' %}
---
In the previous lecture we created a gazebo simulation model of a custom robot. This lecture will continue from that point, by going through the process of adding a controller interface which makes the model movable through ROS.

## General Setup
**Before you do this setup, make sure you have done the setup from the previous lecture (Building a Custom Robot Simulation) first.**

1. Inside the ROS package that you created for building your robot simulation model, create a new folder called `config`.
2. Inside the `config` folder create a new file called `controller_config.yaml`.
3. In the **setup.py** file of your package add the following line:
```python
# In order to be able to access the added files during runtime we add we add them to data_files
# ADD AFTER: ('share/' + package_name, ['package.xml']),
(os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
```
4. Create new file in the `urdf` folder of your ROS package called `robot_control.gazebo.xacro`.
5. If you want to use the example robot from the lecture, copy the `robot_description.urdf.xacro` file from the **ros2_students/custom_robot_sim/urdf** repository and paste it into the urdf folder of your ROS package. In case you already have a robot_description file in your urdf folder either rename the old file or replace it with the new file.
6. Inside the `robot_description.urdf.xacro` file add a reference to the newly created `robot_control.gazebo.xacro` (rember to add this line after the `<robot>` tag):

```xml
<xacro:include filename="$(find PACKAGE_NAME)/urdf/robot_control.gazebo.xacro" />
```

## Robot Arm

The base structure of the `robot_control.gazebo.xacro` file should look like this:
```xml
<?xml version="1.0"?>
<robot>
    <!--Add gazebo specific definitions here-->
</robot>
```

### Gazebo Config
Each joint that you plan to use a position command interface with should have the following definition:
```xml
<gazebo reference="JOINT_NAME">
    <implicitSpringDamper>true</implicitSpringDamper>
</gazebo>
```



We want to be able to move the joints of the robot using ROS2 and therefore define that we want to use the `gazebo_ros2_control` plugin. As a parameter we have to define the location of the `.yaml` file that contains configurations for the controller.

```xml
<gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
        <robot_sim_type>gazebo_ros2_control/GazeboSystem</robot_sim_type>
        <parameters>$(find PACKAGE_NAME)/config/FILENAME.yaml</parameters>
    </plugin>
</gazebo>
```

We also have to define what joints will be commandeble by `ros2_control`. The basic structure of this definition looks like this:
```xml
<ros2_control name="GazeboSystem" type="system">
    <hardware>
        <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>

    <!--Add joint command/state interface definitions-->

</ros2_control>
```

For each joint we then add a definition of the command and state interfaces available:

```xml
<joint name="JOINT_NAME">
    <command_interface name="position">
        <param name="min">-${pi/2}</param>
        <param name="max">${pi/2}</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
</joint>
```

#### Gripper
Another example of defining the command interface of a joint is that we can make one joint simpli mimic another e.g. for a gripper where both fingers are supposed to move together:

```xml
<joint name="gripper_finger_left_joint">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
</joint>

<joint name="gripper_finger_right_joint">
    <param name="mimic">gripper_finger_left_joint</param>
    <param name="multiplier">1</param>
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
</joint>
```



### Controller Config
What type of controller will be used on what joints can be defined in your `controller_config.yaml`. The different types of controllers that are available by default can be found [here](https://control.ros.org/foxy/doc/ros2_controllers/doc/controllers_index.html). First we have to define for the controller manager what type of controllers we want to use: 

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # H

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    CONTROLER_NAME:
      type: joint_trajectory_controller/JointTrajectoryController

    CONTROLER_NAME:
      type: forward_command_controller/ForwardCommandController
```

#### Joint Trajectory Controller
An example difinition of a joint trajectory controller
```yaml
CONTROLER_NAME:
  ros__parameters:
    joints:
      - JOINT_NAME
      - JOINT_NAME
      # ...

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity
```
#### Gripper Controller
An example definition of a forward command controller:
```yaml
CONTROLER_NAME:
  ros__parameters:
    joints:
      - JOINT_NAME

    interface_name: position
```

### Launch File adjustments
To startup your controllers during launch, add the following lines to the launch file for each controller definition:
```python
#ADD IN THE BEGINNING OF THE FILE
from launch.actions import ExecuteProcess

#ADD AFTER: def generate_launch_description():
#BUT BEFORE: return LaunchDescription([
LOAD_CONTROLLER_VARIABLE_NAME = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'start','CONTROLLER_NAME'],
    output='screen'
)

#ADD AFTER: return LaunchDescription([
    LOAD_CONTROLLER_VARIABLE_NAME,
```



## Mobile Platform
Depending on what type of robot you have this part more or less tricky. For a robot with 2 or 4 wheels you can use the plugins in Sections Differential Drive and Skid Steer Drive respectivly. If you have a drone or boat, gazebo classic does not have standard plugins for you. In this case you have 2 options:

1. Install and setup the [PX4 Gazebo Classic Simulations](https://docs.px4.io/main/en/sim_gazebo_classic/). They different implementations already available, but the setup process can be time consuming.
2. Abstract the movement behavoiur of your model e.g. if you have a boat add wheels to it to move it around or if you have a drone add 3 primatic joint to move the drone around.

### Increasing Friction

If you want to change friction values for a link (e.g. for wheels) you can use the following code snippet:

```xml
<gazebo reference="LINK_NAME">
    <mu1>0.1</mu1>
    <mu2>0.1</mu2>
    <kp>500000.0</kp>
    <kd>10.0</kd>
    <minDepth>0.001</minDepth>
    <maxVel>0.1</maxVel>
    <fdir1>1 0 0</fdir1>
    <!-- <material>Gazebo/FlatBlack</material> -->
</gazebo>
```

### Differential Drive

```xml
<gazebo>
    <plugin name="mobile_base_controller" filename="libgazebo_ros_diff_drive.so">
        <ros>
            <!-- <namespace>/demo</namespace>

            <remapping>cmd_vel:=cmd_demo</remapping>
            <remapping>odom:=odom_demo</remapping> -->
        </ros>

        <update_rate>100</update_rate>

        <!--1 for differential drive; 2 for skid steer drive-->
        <num_wheel_pairs>1</num_wheel_pairs>

        <left_joint>front_left_wheel_joint</left_joint>
        <right_joint>front_right_wheel_joint</right_joint>

        <wheel_separation>0.51</wheel_separation>

        <wheel_diameter>0.2</wheel_diameter>

        <max_wheel_torque>50</max_wheel_torque>
        <max_wheel_acceleration>1.0</max_wheel_acceleration>

        <publish_odom>true</publish_odom>
        <publish_odom_tf>true</publish_odom_tf>
        <publish_wheel_tf>false</publish_wheel_tf>

        <odometry_frame>odom</odometry_frame>
        <robot_base_frame>mobile_base_link</robot_base_frame>
    </plugin>
</gazebo>
```

### Skid Steer Drive

```xml
<gazebo>
    <plugin name="mobile_base_controller" filename="libgazebo_ros_diff_drive.so">
        <ros>
            <!-- <namespace>/demo</namespace>

            <remapping>cmd_vel:=cmd_demo</remapping>
            <remapping>odom:=odom_demo</remapping> -->
        </ros>

        <update_rate>100</update_rate>

        <!--1 for differential drive; 2 for skid steer drive-->
        <num_wheel_pairs>2</num_wheel_pairs>

        <left_joint>front_left_wheel_joint</left_joint>
        <right_joint>front_right_wheel_joint</right_joint>

        <left_joint>back_left_wheel_joint</left_joint>
        <right_joint>back_right_wheel_joint</right_joint>

        <wheel_separation>0.51</wheel_separation>
        <wheel_separation>0.51</wheel_separation>

        <wheel_diameter>0.2</wheel_diameter>
        <wheel_diameter>0.2</wheel_diameter>

        <max_wheel_torque>50</max_wheel_torque>
        <max_wheel_acceleration>1.0</max_wheel_acceleration>

        <publish_odom>true</publish_odom>
        <publish_odom_tf>true</publish_odom_tf>
        <publish_wheel_tf>false</publish_wheel_tf>

        <odometry_frame>odom</odometry_frame>
        <robot_base_frame>mobile_base_link</robot_base_frame>
    </plugin>
</gazebo>
```