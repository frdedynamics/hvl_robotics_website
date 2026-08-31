In this tutorial, you will learn about simulation in ROS using Gazebo. You will also learn briefly the physical andy dynamic (inertia and collision) properties of an object and how to create a simulated world.

# Gazebo

Gazebo is the most used simulator in ROS. It has integrated physics rules like gravity, colision, light distribution with reflection and refraction properties, mass and inertia etc. Now, we will see how to set the Gazebo for very basic usage.

{: .notice--info}
You should always remember. Although RViz and Gazebo seems pretty similar, they are used in completely different purposes. Rviz is for visualization (of robots, sensors etc.), Gazebo is for simulation (of robots, objects, environment etc.).

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/gazebo_rr_robot.png)

{: .notice--info} 
Before continuing this tutorial please run these commands:

  ```
  sudo apt update
  sudo apt upgrade
  ```

## Modify XACRO
With the current URDF model we cannot visualize our robot in Gazebo because we haven’t defined any physical properties. We need to improve the URDF model by adding some physical and materialistic properties. To do that, we will create a new XACRO file and a new launch file. Again, we could just modify the ones we already have, but it is nice to compare to see differences.

1. Create a new XACRO file in the urdf folder: `touch ~/ros2_ws/src/my_robotarm_pkg/urdf/my_robotarm_gazebo.xacro`.
1. Create a new launch file in the launch folder: `touch ~/ros2_ws/src/my_robotarm_pkg/launch/my_robotarm_gazebo.launch.py`.
1. Find the relevant files in the Github repo in the [my_robotarm_pkg](https://github.com/frdedynamics/ros2_students_25/tree/master/my_robotarm_pkg/) and copy-paste the content.
1. Observe that each link in the new XACRO file have a <collision/> and <inertial/> tags as well as the <visual/> tag which we had before.
1. Observe that in the new launch file we removed the *node_joint_state_publisher_gui* and put *gazebo_sim* to start the Gazebo simulator, *node_spawn_entity* to spawn the robot in it, and *node_ros_gz_bridge* to connecting ROS and Gazebo to be able to control our robot later on. The reason why we removed the joint state publisher is because now the Gazebo publishes the joint states from the actual *physical* robot state.

We are ready to run. Run these in your ~/ros2_ws directory.
```
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch my_robotarm_pkg my_robotarm_gazebo.launch.py
```

Finally, you should be able to see your colorful RR-manipulator in both RViz and Gazebo Sim. Note that you might need to add a new robot model in RViz to see your robot.

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/gazebo_rviz_rr_robot.png)

## Adding controller

If you realized, your robot has started at the zero-position (vertical) and then fell to this bent position. The reason is that there is no controllers attached to the joints yet. There are different ways to control a robot manipulators; position control, velocity control etc. For this example we will do a *position control in joint space*; meaning that we will give joint angle values to make the robot what we want.

Currently, your Xacro file only defines the kinematic/visual geometry and the Gazebo joint state publisher (which reads joint angles). Your launch file only bridges simulation time and joint states back to ROS.

To command joint movements (position, velocity, or effort), you need to set up ROS 2 Control (``ros2_control``) alongside Gazebo's control plugin.

### Step 1: Add <ros2_control> to your Xacro file
Add the hardware interface definition and Gazebo control plugin to your ``my_robotarm_gazebo.xacro`` file (just before </robot>):

*~/ros2_ws/src/my_robotarm_pkg/urdf/my_robotarm_gazebo.xacro*

```xml
<!-- ROS 2 Control Hardware Interface -->
  <ros2_control name="GazeboSimSystem" type="system">
    <hardware>
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>

    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>

    <joint name="joint2">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>

    <joint name="right_finger_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>

  <!-- Gazebo ROS 2 Control Plugin -->
  <gazebo>
    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimControlPlugin">
      <parameters>$(find my_robotarm_pkg)/config/controllers.yaml</parameters>
    </plugin>
  </gazebo>
  ```

### Step 2: Create a Controller Configuration File
Create a file named ``config/controllers.yaml`` inside ``my_robotarm_pkg``:
```
cd ~/ros2_ws/my_robotarm_pkg
mkdir config
touch controllers.yaml
```
Now paste the content below in the `controllers.yaml`:

*~/ros2_ws/src/my_robotarm_pkg/config/controllers.yaml*
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100 # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

arm_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - right_finger_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

### Step 3: Spawn Controllers in Your Launch File
Add controller spawners to your launch file so ROS 2 automatically loads and starts them when Gazebo opens:

*~/ros2_ws/src/my_robotarm_pkg/launch/my_robotarm_gazebo.launch.py*
```python
  # 1. Joint State Broadcaster Node
    node_joint_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # 2. Arm Controller Node
    node_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )
```
Don't forget to add ``node_joint_broadcaster`` and ``node_arm_controller`` to your ``LaunchDescription[...]`` list in the end.

We are ready to run. Since we created a new folder and file, we must re-compile the whole workspace. Run these in your ~/ros2_ws directory.
```
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch my_robotarm_pkg my_robotarm_gazebo.launch.py
```

Finally run the necessary commands and observe on the terminal that the controllers are loaded successfully.

### Step 4: Sending joint commands to your robot
You can either directly send joint commands to your robot via terminal or write a ROS-node for it. Here I will give you the terminal command and you can try the ROS-node version at home yourself.

Open a new terminal and paste this command:
```
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['joint1', 'joint2', 'right_finger_joint'],
    points: [
      { positions: [1.0, 0.5, 0.02], time_from_start: { sec: 2, nanosec: 0 } }
    ]
  }
}"
```

Note that the way we command the robot is not through a simple topic; we use action/client structure. This subject will be covered later on.






<!-- 


## Adding sensors

<!-- https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Camera -->
<!-- https://gist.github.com/mathewthomas916 -->

To add sensors, we will use gazebo plugins. Camera is probably one of the most common sensor. Let's add camera:

1.Tell Gazebo to use ros2_control. This is also done in  */urdf/gazebo_control.xacro*.
```xml
<!--link : https://www.raspberrypi.org/documentation/hardware/camera/-->
  <gazebo reference="camera_rgb_frame">
    <sensor name="camera" type="camera">
        <always_on>true</always_on>
        <visualize>true</visualize>
        <update_rate>30</update_rate>
        <camera name="picam">
          <horizontal_fov>1.085595</horizontal_fov>
          <image>
              <width>640</width>
              <height>480</height>
              <format>R8G8B8</format>
          </image>
          <clip>
              <near>0.03</near>
              <far>100</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <!-- Noise is sampled independently per pixel on each frame.
                  That pixel's noise value is added to each of its color
                  channels, which at that point lie in the range [0,1]. -->
            <mean>0.0</mean>
            <stddev>0.007</stddev>
          </noise>
        </camera>
          <plugin name="camera_driver" filename="libgazebo_ros_camera.so">
            <ros>
              <!-- <namespace>test_cam</namespace> -->
              <!-- <remapping>image_raw:=image_demo</remapping> -->
              <!-- <remapping>camera_info:=camera_info_demo</remapping> -->
            </ros>
            <!-- camera_name>omit so it defaults to sensor name</camera_name-->
            <!-- frame_name>omit so it defaults to link name</frameName-->
            <!-- <hack_baseline>0.07</hack_baseline> -->
          </plugin>
      </sensor>
    </gazebo>
```
2.It is already added a camera link to your URDF (*/urdf/my_mobile_robot_sensor.xacro*):

```xml
  <joint name="camera_joint" type="fixed">
    <origin xyz="0.073 -0.011 0.084" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="camera_link"/>
  </joint>

  <link name="camera_link">
    <collision>
      <origin xyz="0.005 0.011 0.013" rpy="0 0 0"/>
      <geometry>
        <box size="0.015 0.030 0.027"/>
      </geometry>
    </collision>
  </link>

  <joint name="camera_rgb_joint" type="fixed">
    <origin xyz="0.003 0.011 0.009" rpy="0 0 0"/>
    <parent link="camera_link"/>
    <child link="camera_rgb_frame"/>
  </joint>
  <link name="camera_rgb_frame"/>

  <joint name="camera_rgb_optical_joint" type="fixed">
    <origin xyz="0 0 0" rpy="-1.57 0 -1.57"/>
    <parent link="camera_rgb_frame"/>
    <child link="camera_rgb_optical_frame"/>
  </joint>
  <link name="camera_rgb_optical_frame"/>
```

We are ready to run. Run these in your ~/ros2_ws directory.
```
colcon build
ros2 launch  my_robot_pkg my_mobile_robot.launch.py
```

- `ros2 run rqt_image_view rqt_image_view`
- Observe on Rviz
- Put some obstacles.

{: .notice--info}
**Note:** If your robot doesn't spawn in Gazebo, but is was spawning before, you might be missing some ros2_control packages. Please run these commands in your terminal: `sudo apt install ros-foxy-ros2-control*` and `sudo apt install ros-foxy-gazebo-*`


## Adding world

Currently, the world is empty and meaningless (*smile here*). We can add lots of predefined or custom models into our world.

The following code piece shows how to spawn your robot in a world with `gazebo_ros` node.
```python
  # Define your world path
  world_file_path = os.path.join(get_package_share_directory(my_robot_pkg), 'worlds', 'my_world.world')

  # Starting Gazebo with world
      gazebo = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
          launch_arguments={'world': world_file_path}.items()
      )
```

And a simple world file looks like this:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">

  <world name="default">  

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Global light source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Focus camera on tall pendulum -->
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>4.927360 -4.376610 3.740080 0.000000 0.275643 2.356190</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

  </world>
</sdf>
```

We can add some items by just adding items as we included the sun and the ground plane into our world file. There are many ready-to-use models in the official [gazebo-models](https://github.com/osrf/gazebo_models) repository. Now, choose a model and as in the following code.

```xml
  <!-- Add an object -->
      <include>
        <uri>model://postbox</uri>
      </include>
```

You can change the position and orientation of any object which you have added with the `<pose>x y z roll pitch yaw</pose>` tag. Probably the easiest is to design the world in Gazebo and directly save there.

{: .notice--info}
Don't forget to add your /world folder directory in the **setup.py**

# Troubleshooting

## Cannot do "sudo apt update"
The problem is either about your internet connection or ROS 2 GPG Key. First make sure that you have internet. Afterwards open a new terminal and paste this command: `sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg`


Now you can:
```
sudo apt update
sudo apt upgrade
```

## Getting "cache lock" error
It might be your **lock-frontend** process blocking the system. Find its process id (PID) and kill the process:

```
sudo lsof /var/lib/dpkg/lock-frontend
```

Use this PID and replace \<PID\> in the next command:

```
sudo kill -9 <PID>
```

## Robot looks weird on Gazebo
That is such an annoying problem but let's try. Probably your ros2-control packages are outdated and ´sudo apt upgrade´ cannot fix them - IDK why. We can remove them and reinstall them. Try using these commands:

```
sudo apt remove  ros-foxy-ros2-control*
sudo apt remove ros-foxy-gazebo-*
sudo apt update
sudo apt upgrade
sudo apt install ros-foxy-ros2-control*
sudo apt install ros-foxy-gazebo-*
```

and then restart the VM. -->