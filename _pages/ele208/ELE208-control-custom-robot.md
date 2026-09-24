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

## Basic File Structure

The base structure of the `robot_control.gazebo.xacro` file should look like this:
```xml
<?xml version="1.0"?>
<robot>
    <!--Add gazebo specific definitions here-->
</robot>
```

## Gazebo Config
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
    <command_interface name="position"/>
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



## Controller Config
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


### Forward Command Controller
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

### Joint Trajectory Controller
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

## Launch File adjustments
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

## Controlling the Robot through MATLAB
To control you robot through matlab you will find the example script `robot_arm_pub.m` in the **ros2_students_25/custom_robot_sim** repository. Instructions on how to setup the matlab to be able to communicate through ROS you can find [here](https://frdedynamics.github.io/hvl_robotics_website/courses/ele208/ros-matlab).


### Forward Command Controller
```matlab
robot_arm_controller_node = ros2node("/robot_arm_matlab_controller", 24);

robot_arm_publisher = ros2publisher(robot_arm_controller_node, "/forward_controller/commands", "std_msgs/Float64MultiArray");
joint_states_subscriber = ros2subscriber(robot_arm_controller_node, "/joint_states", "sensor_msgs/JointState");

zero_pos_msg = ros2message(robot_arm_publisher);
zero_pos_msg.data = [0.0 0.0 0.0 0.0 0.0 0.0];

target_pos_msg = ros2message(robot_arm_publisher);
target_pos_msg.data = [pi/4 pi/4 pi/4 pi/4 pi/4 0.07];

move_to_target = false;

for cnt = 1:200
    if mod(cnt, 20) == 0
        move_to_target = ~move_to_target;
    end

    receivedData = receive(joint_states_subscriber, 10);
    joint_names = receivedData.name
    joint_positions = receivedData.position
    joint_velocities = receivedData.velocity
    joint_efforts = receivedData.effort


    if move_to_target
        send(robot_arm_publisher,target_pos_msg);
    else
        send(robot_arm_publisher,zero_pos_msg);
    end
    pause(0.1)
end
```


### Joint Trajectory Controller
```matlab
% Set the ros domain id and create a new ros node
setenv("ROS_DOMAIN_ID","24");
test_node = ros2node("joint_trajectory_client_node");

% Define joint names
joints = { ...
    'arm_base_joint', ...
    'link_1_joint', ...
    'link_2_joint', ...
    'link_3_joint', ...
    'gripper_base_joint', ...
    'gripper_finger_left_joint' };

% Define the action type and name
actionType = "control_msgs/FollowJointTrajectory";
actionName = "/arm_controller/follow_joint_trajectory";


% Create the client
client = ros2actionclient(test_node, actionName, actionType);

disp("Waiting for FollowJointTrajectory action server ...");
waitForServer(client);
disp("Action server available.");

% Define a list of goal positions that will define the trajectory
goalPositions = [ ...
    pi/2, pi/2-0.5, 0.4, pi/4, 0.0, -0.05;    % home-ish
    pi/2, pi/2-0.18, 0.6, pi/4, 0.0, -0.05;  % pose 2
    pi/2, pi/2-0.18, 0.6, pi/4, 0.0, -0.02; % pose 3
    pi/2, pi/2-0.5, 0.4, pi/4, 0.0, -0.02;
    -pi/2, pi/2-0.5, 0.6, pi/4, 0.0, -0.02;
    -pi/2, pi/2-0.35, 0.7, pi/4, 0.0, -0.02;
    -pi/2, pi/2-0.35, 0.7, pi/4, 0.0, -0.05;
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0]; % pose 4

% Create a JointTrajectory message object
traj = ros2message("trajectory_msgs/JointTrajectory");
traj.joint_names = joints;

% Add all the previously defined goalPositions to the trajectory as
% JointTrajectoryPoint
for i = 1:size(goalPositions,1)
    pt1 = ros2message("trajectory_msgs/JointTrajectoryPoint");
    pt1.positions      = [goalPositions(i,:)];  % in radians
    pt1.velocities     = zeros(1,6);
    pt1.accelerations  = zeros(1,6);
    pt1.time_from_start.sec = int32(i);
    pt1.time_from_start.nanosec = uint32(0);
    pt1.effort = [];
    pt1.accelerations = [];

    if i == 1
        traj.points = [pt1];
    else
        traj.points = [traj.points, pt1];
    end
end

% Define the goal message using the previously defined trajectory.
goalMsg = ros2message(client);
goalMsg.trajectory = traj;
goalMsg.goal_time_tolerance.sec     = int32(0);
goalMsg.goal_time_tolerance.nanosec = uint32(500000000);  % 0.5 s

% Send goal
callbackOpts = ros2ActionSendGoalOptions(FeedbackFcn=@feedbackCallback,ResultFcn=@resultCallback);
goalHandle = sendGoal(client, goalMsg, callbackOpts);


function feedbackCallback(goalStruct,feedbackMsg)
    disp('Feedback:')
    disp('Joint Positions:')
    disp(feedbackMsg.desired.positions);
    disp('Joint Velocities:')
    disp(feedbackMsg.desired.velocities);
end

function resultCallback(goalStruct, resultMsg)
    disp('The robot finished the joint trajectory!')
end
```