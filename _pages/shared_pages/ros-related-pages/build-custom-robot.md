In this Chapter we will look at how to build a gazebo model of a robot given the mechanical design. The following topics will be covered:

* Building up the model in Gazebo
* Adding a ROS control interface to the model

## Building a Gazebo Model

A model in Gazebo is made of links which are connected through joints. Since this is a ROS lecture, we use URDF files to declare the model. There are other ways of doing it, which will not be covered. **It is important to notice that all distances are generally in meters and all angles in radian.**

### Link definition

The official documentation of how to define a link in urdf can be found [here](http://wiki.ros.org/urdf/XML/link).

#### Box

```xml
<xacro:property name="length" value="1.0" />
<xacro:property name="width" value="1.0" />
<xacro:property name="height" value="1.0" />
<xacro:property name="density" value="1" />
<xacro:property name="mass" value="${length*height*width*density}" />
<link name="link_name">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="${length} ${width} ${height}"/>
    </geometry>
  </visual>
  <collision>
    <origin xyz="0 0 0.0" rpy="0 0 0"/>
    <geometry>
      <box size="${length} ${width} ${height}"/>
    </geometry>
  </collision>
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="${mass}"/>
    <inertia
      ixx="${mass*(pow(height,2) + pow(width,2))/12}" ixy="0.0" ixz="0.0"
      iyy="${mass*(pow(length,2) + pow(height,2))/12}" iyz="0.0"
      izz="${mass*(pow(length,2) + pow(width,2))/12}"/>
  </inertial>
</link>
```

#### Box Macro Definition

```xml
<xacro:macro name="link_box" params="link_name length width height density:=1 mesh_name:='nofile' *origin_vis *origin_col *origin_inertial">
  <xacro:property name="mass" value="${length*height*width*density}" />
  <link name="${link_name}">
    <visual>
      <!-- <origin xyz="0 0 0" rpy="0 0 0"/> -->
      <xacro:insert_block name="origin_vis" />
      <geometry>
        <xacro:if value="${mesh_name == 'nofile'}">
          <box size="${length} ${width} ${height}"/>
        </xacro:if>
        <xacro:unless value="${mesh_name == 'nofile'}">
          <mesh filename="${mesh_name}" />
        </xacro:unless>
      </geometry>
    </visual>
    <collision>
      <!-- <origin xyz="0 0 0.0" rpy="0 0 0"/> -->
      <xacro:insert_block name="origin_col" />
      <geometry>
        <box size="${length} ${width} ${height}"/>
      </geometry>
    </collision>
    <inertial>
      <!-- <origin xyz="0 0 0" rpy="0 0 0"/> -->
      <xacro:insert_block name="origin_inertial" />
      <mass value="${mass}"/>
      <inertia
        ixx="${mass*(pow(height,2) + pow(width,2))/12}" ixy="0.0" ixz="0.0"
        iyy="${mass*(pow(length,2) + pow(height,2))/12}" iyz="0.0"
        izz="${mass*(pow(length,2) + pow(width,2))/12}"/>
    </inertial>
  </link>
</xacro:macro>
```

#### Box Macro Usage

```xml
<xacro:link_box link_name="LINK_NAME" length="0.1" width="0.1" height="0.1" density="1" mesh_name="nofile">
  <!-- origin visual -->
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <!-- origin collision -->
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <!-- origin interia -->
  <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:link_box>
```

#### Cylinder

```xml
<xacro:property name="radius" value="1.0" />
<xacro:property name="length" value="1.0" />
<xacro:property name="density" value="1.0" />
<xacro:property name="mass" value="${pi*length*pow(radius,2)*density}" /> 

<link name="LINK_NAME">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <cylinder length="${length}" radius="${radius}"/>
    </geometry>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <cylinder length="${length}" radius="${radius}"/>
    </geometry>
  </collision>
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="${mass}"/>
    <inertia
      ixx="${mass*(3*pow(radius,2)+pow(length,2))/12}" ixy="0.0" ixz="0.0"
      iyy="${mass*(3*pow(radius,2)+pow(length,2))/12}" iyz="0.0"
      izz="${mass*pow(radius,2)/2}"/>
  </inertial>
</link>
```

#### Cylinder Macro Definition

```xml
<xacro:macro name="link_cylinder" params="link_name radius length density:=1 mesh_name:='nofile' *origin_vis *origin_col *origin_inertial">
  <xacro:property name="mass" value="${pi*length*pow(radius,2)*density}" />
  <link name="${link_name}">
    <visual>
      <!-- <origin xyz="0 0 0" rpy="0 0 0"/> -->
      <xacro:insert_block name="origin_vis" />
      <geometry>
        <xacro:if value="${mesh_name == 'nofile'}">
          <cylinder length="${length}" radius="${radius}"/>
        </xacro:if>
        <xacro:unless value="${mesh_name == 'nofile'}">
          <mesh filename="${mesh_name}" />
        </xacro:unless>
      </geometry>
    </visual>
    <collision>
      <!-- <origin xyz="0 0 0" rpy="0 0 0"/> -->
      <xacro:insert_block name="origin_col" />
      <geometry>
        <cylinder length="${length}" radius="${radius}"/>
      </geometry>
    </collision>
    <inertial>
      <!-- <origin xyz="0 0 0" rpy="0 0 0"/> -->
      <xacro:insert_block name="origin_inertial" />
      <mass value="${mass}"/>
      <inertia
        ixx="${mass*(3*pow(radius,2)+pow(length,2))/12}" ixy="0.0" ixz="0.0"
        iyy="${mass*(3*pow(radius,2)+pow(length,2))/12}" iyz="0.0"
        izz="${mass*pow(radius,2)/2}"/>
    </inertial>
  </link>
</xacro:macro>
```

#### Cylinder Macro Usage

```xml
<xacro:link_cylinder link_name="LINK_NAME" radius="0.1" length="0.1" density="1" mesh_name="nofile" >
  <!-- origin visual -->
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <!-- origin collision -->
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <!-- origin interia -->
  <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:link_cylinder>
```

#### Sphere

```xml
<xacro:property name="radius" value="1.0" />
<xacro:property name="density" value="1.0" />
<xacro:property name="mass" value="${(4/3)*pi*pow(radius,3)}" />
<link name="LINK_NAME">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <sphere radius="${radius}"/>
    </geometry>
  </visual>
  <collision>
    <origin xyz="0 0 0.0" rpy="0 0 0"/>
    <geometry>
      <sphere radius="${radius}"/>
    </geometry>
  </collision>
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="${mass}"/>
    <inertia
      ixx="${2*mass*pow(radius,2)/5}" ixy="0.0" ixz="0.0"
      iyy="${2*mass*pow(radius,2)/5}" iyz="0.0"
      izz="${2*mass*pow(radius,2)/5}"/>
  </inertial>
</link>
```

#### Sphere Macro Definition

```xml
<xacro:macro name="link_sphere" params="link_name radius density:=1 mesh_name:='nofile' *origin_vis *origin_col *origin_inertial">
  <xacro:property name="mass" value="${(4/3)*pi*pow(radius,3)}" />
  <link name="${link_name}">
    <visual>
      <!-- <origin xyz="0 0 0" rpy="0 0 0"/> -->
      <xacro:insert_block name="origin_vis" />
      <geometry>
        <xacro:if value="${mesh_name == 'nofile'}">
          <sphere radius="${radius}"/>
        </xacro:if>
        <xacro:unless value="${mesh_name == 'nofile'}">
          <mesh filename="${mesh_name}" />
        </xacro:unless>
      </geometry>
    </visual>
    <collision>
      <!-- <origin xyz="0 0 0.0" rpy="0 0 0"/> -->
      <xacro:insert_block name="origin_col" />
      <geometry>
        <sphere radius="${radius}"/>
      </geometry>
    </collision>
    <inertial>
      <!-- <origin xyz="0 0 0" rpy="0 0 0"/> -->
      <xacro:insert_block name="origin_inertial" />
      <mass value="${mass}"/>
      <inertia
        ixx="${2*mass*pow(radius,2)/5}" ixy="0.0" ixz="0.0"
        iyy="${2*mass*pow(radius,2)/5}" iyz="0.0"
        izz="${2*mass*pow(radius,2)/5}"/>
    </inertial>
  </link>
</xacro:macro>
```

#### Sphere Macro Usage

```xml
<xacro:link_sphere link_name="LINK_NAME" radius="0.1" density="1" mesh_name="nofile">
  <!-- origin visual -->
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <!-- origin collision -->
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <!-- origin interia -->
  <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:link_sphere>
```

#### Mesh

```xml
<geometry>
  <mesh filename="package://custom_robot_tutorial/meshes/wheel_mesh.stl" />
</geometry>
```

#### Inertial Parameters

An accurate simulation requires accurate dynamic properties of the links. These are defined by the inertial parameters. An official guide on the Gazebo website can be found [here](http://gazebosim.org/tutorials?tut=inertia). An online tool (mesh cleaner) to calculate the inertial parameters for mesh files automatically can be found [here](https://www.hamzamerzic.info/mesh_cleaner/). When using simple shapes, the previous example code for the different links automatically calculates the inertial parameters assuming you use the defined variables. A list of formulas to calculate the inertial parameters for simple shapes can be found [here](https://en.wikipedia.org/wiki/List_of_moments_of_inertia).

**Make sure the center of mass is in the accurate location.** For simple shapes it is in the center of the link, which means, as long as the `<origin>` inside the `<inertial>` tag is the same as the `<origin>` in the `<collision>` tag it should be correct.

### Joint definition

The official documentation of how to define a joint in urdf can be found [here](http://wiki.ros.org/urdf/XML/joint). Some important parts of the joint definition are the following:

- `<axis>` defines which axis is used by the joint for movement.
- `<limit>` 
    - `effort` maximum torque/force measured in [Nm]
    - `velocity` maximum speed measured in [m/s] for primatic joints and [rad/s] for revolute joints
    - `lower` minimum allowed joint angle/position measured in [m] for prismatic joints and [rad] for revolute joints.
    - `upper` maximum allowed joint angle/position

#### Continuous Joint

Continuous joints rotate around the defined axis (1 degree of freedom).

```xml
<joint type="continuous" name="JOINT_NAME">
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <child link="CHILD_LINK_NAME"/>
  <parent link="PARENT_LINK_NAME"/>
  <axis xyz="0 0 1" rpy="0 0 0"/>
  <limit effort="100" velocity="100"/>
</joint>
```

#### Revolute Joint

Revolute joints also rotate around the defined axis (1 degree of freedom) similar to continuous joints but have a defined minimum and maximum joint angle in [rad].

```xml
<joint type="revolute" name="JOINT_NAME">
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <child link="CHILD_LINK_NAME"/>
  <parent link="PARENT_LINK_NAME"/>
  <axis xyz="0 0 1" rpy="0 0 0"/>
  <limit effort="100" velocity="100" lower="-${pi}" upper="${pi}"/>
</joint>
```
#### Prismatic Joint

Prismatic joints move along the defined axis (1 degree of freedom) and have a minimum and maximum joint position definedin [m].

```xml
<joint type="prismatic" name="JOINT_NAME">
  <origin xyz="0 0 1.0" rpy="0 0 0"/>
  <child link="CHILD_LINK_NAME"/>
  <parent link="PARENT_LINK_NAME"/>
  <axis xyz="0 0 1" rpy="0 0 0"/>
  <limit effort="100" velocity="100" lower="-1.0" upper="1.0"/>
</joint>
```

#### Fixed Joint

Fixed joints are not really joints because all degrees of freedom are blocked.

```xml
<joint name="JOINT_NAME" type="fixed">
  <parent link="PARENT_LINK_NAME"/>
  <child link="CHILD_LINK_NAME" />
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

### Exercise

Download the ROS package used for the exercise from this link. After unzipping it, copy it to the VM in catkin_ws/src/. Run the following commands in a terminal window:

```bash
cd ~/catkin_ws/
catkin_make
```

Copy the following code into the file “mobile_manipulator_robot.urdf.xacro” located in the “urdf” folder of the ROS package:

```xml
<?xml version='1.0'?>

<robot name="mobile_manipulator_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="$(find custom_robot_tutorial)/urdf/common_macros.xacro" />

  <gazebo>
    <static>true</static>
  </gazebo>

<!--############################### -->
<!-- MOBILE PLATFORM -->
<!--############################### -->
  <origin xyz="0 0 0.15" rpy="0 0 0"/>
  <link name="base_footprint"/>

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="mobile_body_link" />
    <origin xyz="0 0 0.0" rpy="0 0 0"/>
  </joint>


  <!-- MOBILE BASE -->
  <!-- ==================================== -->
  <xacro:link_box link_name="mobile_body_link" length="0.65" width="0.4" height="0.2" mesh_name="package://custom_robot_tutorial/meshes/base_mesh.stl">
    <!-- origin visual -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <!-- origin collision -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <!-- origin interia -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:link_box>


  <!-- FRONT LEFT WHEEL -->
  <!-- ==================================== -->
  <joint type="continuous" name="wheel_front_left_joint">
    <origin xyz="0.200 0.255 -0.050" rpy="-${pi/2} 0 0"/>
    <child link="wheel_front_left_link"/>
    <parent link="mobile_body_link"/>
    <axis xyz="0 0 1" rpy="0 0 0"/>
    <limit effort="100" velocity="100"/>
  </joint>

  <xacro:link_cylinder link_name="wheel_front_left_link" radius="0.1" length="0.1" density="1" mesh_name="package://custom_robot_tutorial/meshes/wheel_mesh.stl" >
    <!-- origin visual -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <!-- origin collision -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <!-- origin interia -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:link_cylinder>

  <!-- FRONT RIGHT WHEEL -->
  <!-- ==================================== -->
  <joint type="continuous" name="wheel_front_right_joint">
    <origin xyz="0.200 -0.255 -0.050" rpy="-${pi/2} 0 0"/>
    <child link="wheel_front_right_link"/>
    <parent link="mobile_body_link"/>
    <axis xyz="0 0 1" rpy="0 0 0"/>
    <limit effort="100" velocity="100"/>
  </joint>

  <xacro:link_cylinder link_name="wheel_front_right_link" radius="0.1" length="0.1" density="1" mesh_name="package://custom_robot_tutorial/meshes/wheel_mesh.stl" >
    <!-- origin visual -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <!-- origin collision -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <!-- origin interia -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:link_cylinder>

</robot>
```

To start the Gazebo world use the following command in the terminal:

```bash
roslaunch custom_robot_tutorial mobile_manipulator.launch
```

To spawn the robot in the existing Gazebo world use the following terminal command:

```bash
roslaunch custom_robot_tutorial robot_spawn.launch
```

Start adding the missing 3 wheels to the robot model by modifying the “mobile_manipulator_robot.urdf.xacro” file. Where to place the wheels can be deducted from the following mechanical drawings:

![alt]({{ site.url }}{{ site.baseurl }}/assets/images/shared/build_custom_robot/mobile_robot_plan.png)
![alt]({{ site.url }}{{ site.baseurl }}/assets/images/shared/build_custom_robot/robot_arm_plan.png)

Once you finished with adding all the wheels to the mobile platform, copy the following code into the `mobile_manipulator_robot.urdf.xacro` file just before the `</robot>` tag at the end of the file:

```xml
<!--############################### -->
<!-- ROBOTIC ARM -->
<!--############################### -->

  <!-- ARM BASE -->
  <!-- ==================================== -->
  <joint type="revolute" name="arm_base_joint">
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <child link="arm_base_link"/>
    <parent link="mobile_body_link"/>
    <axis xyz="0 0 1" rpy="0 0 0"/>
    <limit effort="100" velocity="100" lower="-${pi}" upper="${pi}"/>
  </joint>

  <xacro:link_cylinder link_name="arm_base_link" radius="${0.135/2}" length="0.2" density="1" mesh_name="package://custom_robot_tutorial/meshes/arm_base_mesh.stl" >
    <!-- origin visual -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <!-- origin collision -->
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <!-- origin interia -->
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </xacro:link_cylinder>

  <!-- LINK 1 -->
  <!-- ==================================== -->
  <joint type="revolute" name="link_1_joint">
    <origin xyz="0 0 0.2" rpy="-${pi/2} 0 0"/>
    <child link="link_1_link"/>
    <parent link="arm_base_link"/>
    <axis xyz="0 0 1" rpy="0 0 0"/>
    <limit effort="100" velocity="100" lower="-${pi/2}" upper="${pi/2}"/>
  </joint>

  <xacro:link_cylinder link_name="link_1_link" radius="${0.075/2}" length="0.385" density="1" mesh_name="package://custom_robot_tutorial/meshes/link_1_mesh.stl" >
    <!-- origin visual -->
    <origin xyz="0 -${0.385/2} 0" rpy="${pi/2} 0 0"/>
    <!-- origin collision -->
    <origin xyz="0 -${0.385/2} 0" rpy="${pi/2} 0 0"/>
    <!-- origin interia -->
    <origin xyz="0 -${0.385/2} 0" rpy="${pi/2} 0 0"/>
  </xacro:link_cylinder>
```

