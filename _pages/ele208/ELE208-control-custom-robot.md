---
layout: single
title: "Control a Custom Robot Simulation"
permalink: /courses/ele208/control-custom-robot
toc: true # The right side bar "on this page"
breadcrumbs: true  # the directory-of-documents type of header navigation
sidebar:
  nav: "ele208"  # the left navigation bar. Choose which category you want.
taxonomy: markup
# {% include_absolute '_pages/shared_pages/ros-related-pages/build-custom-robot.md' %}
---
In the previous lecture we created a gazebo simulation model of a custom robot. This lecture will continue from that point, by going through the process of adding a controller interface which makes the model movable through ROS.

## General Setup
**Before you do this setup, make sure you have done the setup from the previous lecture (Build a Custom Robot Simulation) first.**

1. Inside the ROS package that you created for building your robot simulation model, create a new folder called `config`.
2. Inside the `config` folder create a new file called `controller_config.yaml`.
3. In the **setup.py** file of your package add the following line:
```python
# In order to be able to access the added files during runtime we add we add them to data_files
# ADD AFTER: ('share/' + package_name, ['package.xml']),
(os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
```
4. Create new file in the `urdf` folder of your ROS package called `robot_control.gazebo.xacro`.
<!-- 5. If you want to use the example robot from the lecture, copy the `robot_description.urdf.xacro` file from the **ros2_students_25/custom_robot_sim/urdf** repository and paste it into the urdf folder of your ROS package. In case you already have a robot_description file in your urdf folder either rename the old file or replace it with the new file. -->
<!-- 6. If you want to try the example publishers, copy `mobile_base_pub.py`, `robot_arm_pub.py` and `gripper_pub.py` from the  **ros2_students_25/custom_robot_sim** repository and paste the files into **ros2_ws/src/PACKAGE_NAME/PACKAGE_NAME**. **Note:** in the lecture example PACKAGE_NAME = custom_robot_sim. Also, don't forget to reference the file in `setup.py` in the `console_scripts` section like we did with `reload_robot_model.py` in the [Build a Custom Robot Simulation](https://frdedynamics.github.io/hvl_robotics_website/courses/ele208/build-custom-robot) Setup. -->
7. Inside the `robot_description.urdf.xacro` file add a reference to the newly created `robot_control.gazebo.xacro` (remember to add this line after the `<robot>` tag):

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
We want to be able to move the joints of the robot using ROS2 and therefore define that we want to use the `gz_ros2_control` plugin. As a parameter we have to define the location of the `.yaml` file that contains configurations for the controller.

```xml
<gazebo>
    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
        <parameters>$(find PACKAGE_NAME)/config/FILENAME.yaml</parameters>
    </plugin>
</gazebo>
```

We also have to define what joints will be controllable by `ros2_control`. The basic structure of this definition looks like this:
```xml
<ros2_control name="GazeboSimSystem" type="system">
    <hardware>
        <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>

    <!--Add joint command/state interface definitions-->

</ros2_control>
```

For each joint we then add a definition of the command and state interfaces available:

```xml
<joint name="JOINT_NAME">
    <command_interface name="position">
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
</joint>
```

**Note:** You can define multiple command interfaces in case you want to also be able to control the velocity or effort (torque) of the joint.

<!-- #### Gripper
Another example of defining the command interface of a joint is that we can make one joint simply mimic another e.g. for a gripper where both fingers are supposed to move together:

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
``` -->



### Controller Config
What type of controller will be used on what joints can be defined in your `controller_config.yaml`. The different types of controllers that are available by default can be found [here](https://control.ros.org/jazzy/doc/ros2_controllers/doc/controllers_index.html). First we have to define for the controller manager what type of controllers we want to use: 

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # H

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    CONTROLLER_NAME:
      type: forward_command_controller/ForwardCommandController

    CONTROLLER_NAME:
      type: joint_trajectory_controller/JointTrajectoryController
```


#### Forward Command Controller
An example definition of a forward command controller which uses type `forward_command_controller/ForwardCommandController`:
```yaml
CONTROLLER_NAME:
  ros__parameters:
    joints:
      - JOINT_NAME
      - JOINT_NAME

    interface_name: position
```
**Note:** the interface can also be velocity or effort depending on how the joints should be controlled.

#### Joint Trajectory Controller
The definition of the joint trajectory controller of type `joint_trajectory_controller/JointTrajectoryController` is slightly different to the forward command controller:
```yaml
CONTROLLER_NAME:
  ros__parameters:
    joints:
      - JOINT_NAME
      - JOINT_NAME

    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

### Launch File adjustments
To startup your controllers during launch, add the following lines to the launch file for each controller definition:
```python

#ADD AFTER: def generate_launch_description():
#BUT BEFORE: return LaunchDescription([
node_joint_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
)

# This is the node allows you to control your robot arm.
node_arm_controller = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["CONTROLLER_NAME"],
    output="screen",
)

#ADD AFTER: return LaunchDescription([
    node_joint_broadcaster,
    node_arm_controller,
```

<!-- ## Control the Robot
After doing the configurations described before you should now be able to control your robot through ROS. If you added the example script into your package as described in the [Setup]() section, you can now test controlling the wheels, arm joints and gripper fingers using the following commands. **Note:** don't forget to launch your robot simulation first!

```bash
ros2 run custom_robot_sim mobile_base_pub
```

```bash
ros2 run custom_robot_sim robot_arm_pub
```

```bash
ros2 run custom_robot_sim gripper_pub
``` -->

### Matlab
To control you robot through matlab you will find the example script `robot_arm_pub.m` in the **ros2_students_25/custom_robot_sim** repository. Instructions on how to setup the matlab to be able to communicate through ROS you can find [here](https://frdedynamics.github.io/hvl_robotics_website/courses/ele208/ros-matlab).
