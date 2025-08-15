---
layout: single
title: "Build a Custom Robot Simulation"
permalink: /courses/ele306/build-custom-robot
toc: true # The right side bar "on this page"
breadcrumbs: true  # the directory-of-documents type of header navigation
sidebar:
  nav: "ele306"  # the left navigation bar. Choose which category you want.
taxonomy: markup
# {% include_absolute '_pages/shared_pages/ros-related-pages/build-custom-robot.md' %}
---
In this chapter we will look at how to build a gazebo simulation model of a robot given the mechanical design. The following topics will be covered:
* Building up the model in Gazebo
* Adding a ROS control interface to the model

If you have time to do so, we recommend you to create a [CAD](https://www.autodesk.de/solutions/cad-software) model of your robot before you create the simulation model as it will make this process easier. You can install [Fusion 360](https://www.autodesk.com/education/edu-software/overview?sorting=featured&filters=individual#card-f360) from Autodesk for free if you create an account with you HVL email and [verify your student status](https://www.autodesk.com/support/technical/article/caas/sfdcarticles/sfdcarticles/How-to-verify-your-student-eligibility.html). [Learn Fusion 360 in 90 Minutes](https://www.autodesk.com/certification/learn/course/learn-fusion-360-in-90-minutes) is recommended to get started and if you are interested in more in-depth tutarials you can find them [here](https://www.autodesk.com/certification/learn/catalog/product/Fusion%20360). 

## Setup Process
**If you do this setup process for the lecture example make sure to use `custom_robot_sim` as the package name.**

Before we can start defining our robot model, we have to create and configure a new ROS package:
1. Open a new terminal window by pressing: Ctrl+Alt+T
2. Navigate into the src folder of your ros2 workspace:
```bash
cd ~/ros2_ws/src
```
3. Create a new ROS2 python package (replace PACKAGE_NAME with the name of your package):
```bash
ros2 pkg create --build-type ament_python PACKAGE_NAME 
```
4. Copy the folders **launch**, **meshes** and **urdf** from the **[ros2_students_25](https://github.com/frdedynamics/ros2_students_25)/custom_robot_sim** git repository to your newly created package (ros2_ws/src/PACKAGE_NAME)
5. Copy the **reload_robot_model.py** to ros2_ws/src/PACKAGE_NAME/PACKAGE_NAME
6. In the **setup.py** file of your package add the following lines:
```python
# ADD AFTER: from setuptools import setup
import os
from glob import glob
# In order to be able to access the added files during runtime we add we add them to data_files
# ADD AFTER: ('share/' + package_name, ['package.xml']),
(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
(os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
(os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.stl'))),
# In order to be able to run any of our ros nodes we have to add them
# ADD AFTER: 'console_scripts': [
'reload_robot_model = custom_robot_sim.reload_robot_model:main'
```
6. Create a new file in the **urdf folder** of your package named `robot_description.urdf.xacro`
7. In the **launch/robot_spawn.launch.py** make sure that the **xacro_file_path** is correct.
8. Move to workspace directory, build your workspace and source it:
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash 
```
9. To launch **gazebo** run:
```bash
ros2 launch PACKAGE_NAME start_gazebo.launch.py
```
10. To **spawn the robot model** in gazebo run:
```bash
ros2 launch PACKAGE_NAME robot_spawn.launch.py
```

## Basic Xacro Robot Description structure
A model in Gazebo is made of links which are connected through joints. Since this is a ROS lecture, we use URDF files to describe the model. URDF (Unified Robot Description Format) is a file format for specifying the geometry and organization of robots in ROS. The official guides on using URDF and Xacro can be found [here](https://docs.ros.org/en/foxy/Tutorials/Intermediate/URDF/URDF-Main.html). In this course we will use Xacro which is basically the same as URDF but has some features that can make your life easier:
* **Constants:** Lets you assign a value to a variable that can then be used multiple time:
  ```xml
  <!--Defining a constant-->
  <xacro:property name="width" value="0.2" />
  <!--Example of how to use the defined constant-->
  <sphere radius="${width}"/>
  ```
* **Simple Math:** You can build up arbitrarily complex expressions in the ${} construct using the four basic operations (+,-,*,/), the unary minus, and parenthesis. Functions and constants from the python math module (e.g. `pi` and trigonometric functions like `sin` or  `cos`) are also available for use.
  ```xml
  <origin xyz="0.200 ${width/2} -0.050" rpy="-${pi/2} 0 0"/>
  ```
* **Macros:** This is the main feature of xacro. Define macros with the macro tag, and specify the macro name and the list of parameters. The list of parameters should be whitespace separated. They become macro-local properties.
  ```xml
  <!--Example of a macro definition-->
  <xacro:macro name="default_inertial" params="mass">
    <inertial>
            <mass value="${mass}" />
            <inertia ixx="1e-3" ixy="0.0" ixz="0.0"
                  iyy="1e-3" iyz="0.0"
                  izz="1e-3" />
    </inertial>
  </xacro:macro>
  <!--Example of how to use the previously defined macro-->
  <xacro:default_inertial mass="10"/>
  ```

To use xacro for your robot model description, the basic structure should look like this:
```xml
<?xml version="1.0" ?>

<robot name="ROBOT_NAME" xmlns:xacro="http://ros.org/wiki/xacro">
  <!-- Uncomment to disable the physics engine! -->
  <!-- <gazebo>
    <static>true</static>
  </gazebo> -->

  <!-- ADD LINK AND JOINTS HERE -->
</robot>
```

 **In general it is important to notice that all distances are normally in meters and all angles in radian.**

## Links

### Basic Structure
The official documentation on how to define a link in urdf can be found [here](http://wiki.ros.org/urdf/XML/link).

```xml
<link name="LINK_NAME">
    <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
            <!-- ADD VISUAL GEOMETRY HERE -->
        </geometry>
    </visual>
    <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
            <!-- ADD COLLISION GEOMETRY HERE -->
        </geometry>
    </collision>
    <inertial>
        <!-- ADD INERTIAL PARAMETERS HERE -->
    </inertial>
</link>
```

#### Dummy Links
It is good practice to define the first link of your robot definition as an empty link/dummy link which you then connect your robots' base link to with a fixed joint. You can find more information on why we do this [here](https://answers.ros.org/question/415304/urdf-what-is-a-dummy-link-needed-for/).
```xml
<link name="LINK_NAME"/>
```

### Geometries
The geometry defines the shape of the visual object. This can be a simple shape (box, cylinder or sphere) or a mesh geometry. A mesh uses a combination of triangles to create shapes that are not simple shapes. The more complex the geometry and the higher the resolution the more triangles are needed:

![alt]({{ site.url }}{{ site.baseurl }}/assets/images/ele306/build_custom_robot_model/husky_mesh.png)

#### Box

```xml
<!--size parameters: length_along_x_axis length_along_y_axis length_along_z_axis-->
<box size="0.0 0.0 0.0"/>
```

#### Cylinder

```xml
<cylinder length="0.0" radius="0.0" />
```

#### Sphere

```xml
<sphere radius="0.0"/>
```

#### Mesh

```xml
<mesh filename="$(find PACKAGE_NAME)/meshes/MESH_FILENAME.stl" />
```

### Inertial Parameters
An accurate simulation requires accurate dynamic properties of the links. These are defined by the inertial parameters. An official guide on the Gazebo website can be found [here](https://classic.gazebosim.org/tutorials?tut=inertia). There are multiple ways of calculating these parameters. Here are some options:
* If you designed you robot model in a CAD software you can often read out these parameters for your model. In Autodesk Fusion 360 you can [assign a material to your model](https://help.autodesk.com/view/fusion360/ENU/?guid=GUID-4897BAE8-65BB-48FA-B764-D3749B32F6B2) and then [read out the physical properties](https://www.autodesk.com/support/technical/article/caas/sfdcarticles/sfdcarticles/How-to-Find-Mass-Properties-in-Fusion-360.html).
* The online tool mesh cleaner to calculate the inertial parameters for mesh files automatically can be found [here](https://www.hamzamerzic.info/mesh_cleaner/). Note that this doesn't always work and depends on how complex your shapes are.
* An approximation can also be made by calculating the intertial parameters for a simple shape that is close to the model. You can use the following macros for that. The list of the formulas used in these macros to calculate the inertial parameters can be found [here](https://en.wikipedia.org/wiki/List_of_moments_of_inertia).

**Make sure the center of mass is in the accurate location.** For simple shapes it is in the center of the link, which means, as long as the <origin> inside the <intertail> tag is the same as the <origin> in the <collision> tag it should be correct.

```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <mass value="1.0"/>
  <inertia
    ixx="1.0" ixy="0.0" ixz="0.0"
    iyy="1.0" iyz="0.0"
    izz="1.0"/>
</inertial>
```

#### Box Macro Usage
```xml
<xacro:inertial_box length="0.0" width="0.0" height="0.0" density="1">
  <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:inertial_box>
```

#### Cylinder Macro Usage
```xml
<xacro:inertial_cylinder radius="0.0" length="0.0" density="1">
  <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:inertial_cylinder>
```

#### Sphere Macro Usage
```xml
<xacro:inertial_cylinder radius="0.0" density="1">
  <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:inertial_cylinder>
```

## Joints
The official documentation of how to define a joint in urdf can be found [here](http://wiki.ros.org/urdf/XML/joint). Some important parts of the joint definition are the following:
* `<axis>` defines which axis is used by the joint for movement.
* `<limit>`
  * `effort` maximum torque/force measured in [Nm]
  * `velocity` maximum speed measured in [m/s] for primatic joints and [rad/s] for revolute joints
  * `lower` minimum allowed joint angle/position measured in [m] for prismatic joints and [rad] for revolute joints.
  * `upper` maximum allowed joint angle/position


### Continuous Joint
Continuous joints rotate around the defined axis (1 degree of freedom).
```xml
<joint type="continuous" name="JOINT_NAME">
  <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
  <child link="CHILD_LINK_NAME"/>
  <parent link="PARENT_LINK_NAME"/>
  <axis xyz="0 0 1" rpy="0 0 0"/>
  <limit effort="100" velocity="100"/>
  <dynamics damping="0.7" friction="1.0"/>
</joint>
```

### Revolute Joint
Revolute joints also rotate around the defined axis (1 degree of freedom) similar to continuous joints but have a defined minimum and maximum joint angle in [rad].
```xml
<joint type="revolute" name="JOINT_NAME">
  <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
  <child link="CHILD_LINK_NAME"/>
  <parent link="PARENT_LINK_NAME"/>
  <axis xyz="0 0 1" rpy="0 0 0"/>
  <limit effort="100" velocity="100" lower="-${pi/2}" upper="${pi/2}"/>
  <dynamics damping="0.7" friction="1.0"/>
</joint>
```

### Prismatic Joint
Prismatic joints move along the defined axis (1 degree of freedom) and have a minimum and maximum joint position defined in [m].
```xml
<joint type="prismatic" name="JOINT_NAME">
  <origin xyz="0 0 1.0" rpy="0 0 0"/>
  <child link="CHILD_LINK_NAME"/>
  <parent link="PARENT_LINK_NAME"/>
  <axis xyz="0 0 1" rpy="0 0 0"/>
  <limit effort="100" velocity="100" lower="-1.0" upper="1.0"/>
  <dynamics damping="0.7" friction="1.0"/>
</joint>
```

### Fixed Joint
Fixed joints are not really joints because all degrees of freedom are blocked.
```xml
<joint name="JOINT_NAME" type="fixed">
  <parent link="PARENT_LINK_NAME"/>
  <child link="CHILD_LINK_NAME" />
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

## Include other xacro files
```xml
  <xacro:include filename="$(find PACKAGE_NAME)/urdf/FILENAME.xacro" />
```

## Exercises
Before starting the exercises make sure you have gone through the **setup process** in the beginning of this page. Cope the exercise code into the `robot_description.urdf.xacro` file that you created.
### Mechanical Drawing
![alt]({{ site.url }}{{ site.baseurl }}/assets/images/ele306/build_custom_robot_model/mobile_robot_plan.png)
![alt]({{ site.url }}{{ site.baseurl }}/assets/images/ele306/build_custom_robot_model/robot_arm_plan.png)
### Exercise 1
For the first exercise you are given the model of the mobile base with 2 wheels attached. Your job is to add the remaining 2 wheels. Use the mechanical drawing of the robot for the right placements.
```xml
<?xml version="1.0" ?>

<robot name="mobile_manipulator_robot" xmlns:xacro="http://ros.org/wiki/xacro">
    <xacro:include filename="$(find custom_robot_sim)/urdf/common_macros.xacro" />
    <!-- <xacro:include filename="$(find custom_robot_sim)/urdf/mobile_manipulator_robot.gazebo.xacro" /> -->

    <gazebo>
        <static>true</static>
    </gazebo>

    <xacro:property name="density_pla" value="600" />

    <!--############################### -->
    <!-- MOBILE PLATFORM -->
    <!--############################### -->

    <link name="base_footprint"/>

    <joint name="base_joint" type="fixed">
        <parent link="base_footprint"/>
        <child link="mobile_base_link" />
        <origin xyz="0 0 0.15" rpy="0 0 0"/>
    </joint>

    <!-- MOBILE BASE -->
    <!-- ==================================== -->

    <link name="mobile_base_link">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <mesh filename="$(find custom_robot_sim)/meshes/base_mesh.stl" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <box size="0.650 0.400 0.200"/>
            </geometry>
        </collision>
        <xacro:inertial_box length="0.650" width="0.400" height="0.200" density="${density_pla}">
            <origin xyz="0 0 0" rpy="0 0 0"/>
        </xacro:inertial_box>
    </link>

    <!-- FRONT LEFT WHEEL -->
    <!-- ==================================== -->
    <joint type="continuous" name="front_left_wheel_joint">
        <origin xyz="0.200 0.255 -0.050" rpy="-${pi/2} 0 0"/>
        <child link="front_left_wheel_link"/>
        <parent link="mobile_base_link"/>
        <axis xyz="0 0 1" rpy="0 0 0"/>
        <limit effort="1000" velocity="1000"/>
        <dynamics damping="0.7" friction="1.0"/>
    </joint>

    <link name="front_left_wheel_link">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <mesh filename="$(find custom_robot_sim)/meshes/wheel_mesh.stl" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <cylinder length="0.1" radius="0.1" />
            </geometry>
        </collision>
        <xacro:inertial_cylinder radius="0.1" length="0.1" density="${density_pla}">
            <origin xyz="0 0 0" rpy="0 0 0"/>
        </xacro:inertial_cylinder>
    </link>

    <!-- FRONT RIGHT WHEEL -->
    <!-- ==================================== -->
    <joint type="continuous" name="front_right_wheel_joint">
        <origin xyz="0.200 -0.255 -0.050" rpy="-${pi/2} 0 0"/>
        <child link="front_right_wheel_link"/>
        <parent link="mobile_base_link"/>
        <axis xyz="0 0 1" rpy="0 0 0"/>
        <limit effort="1000" velocity="1000"/>
        <dynamics damping="0.7" friction="1.0"/>
    </joint>

    <link name="front_right_wheel_link">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <mesh filename="$(find custom_robot_sim)/meshes/wheel_mesh.stl" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <cylinder length="0.1" radius="0.1" />
            </geometry>
        </collision>
        <xacro:inertial_cylinder radius="0.1" length="0.1" density="${density_pla}">
            <origin xyz="0 0 0" rpy="0 0 0"/>
        </xacro:inertial_cylinder>
    </link>
</robot>    
```

### Exercise 2
In the second exercise you are given the model description of the first 2 links of the manipulator. Add this description to the code of exercise 1 after the definition of the wheels but **before** the `</robot>` tag. Your job is to add the remaining links. Use the mechanical drawing of the robot for the right placements.
```xml
    <!--############################### -->
    <!-- ROBOTIC ARM -->
    <!--############################### -->

    <!-- ARM BASE -->
    <!-- ==================================== -->
    <joint type="revolute" name="arm_base_joint">
        <origin xyz="0 0 0.1" rpy="0 0 0"/>
        <child link="arm_base_link"/>
        <parent link="mobile_base_link"/>
        <axis xyz="0 0 1" rpy="0 0 0"/>
        <limit effort="100" velocity="100" lower="-${pi}" upper="${pi}"/>
        <dynamics damping="0.7" friction="1.0"/>
    </joint>

    <link name="arm_base_link">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <mesh filename="$(find custom_robot_sim)/meshes/arm_base_mesh.stl" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0.05" rpy="0 0 0"/>
            <geometry>
                <cylinder length="0.1" radius="${0.135/2}" />
            </geometry>
        </collision>
        <collision>
            <origin xyz="0 0 0.15" rpy="0 0 0"/>
            <geometry>
                <box size="0.075 0.075 0.1"/>
            </geometry>
        </collision>
        <xacro:inertial_cylinder radius="${0.135/2}" length="0.2" density="${density_pla}">
            <origin xyz="0 0 0.1" rpy="0 0 0"/>
        </xacro:inertial_cylinder>
    </link>
    

    <!-- LINK 1 -->
    <!-- ==================================== -->
    <joint type="revolute" name="link_1_joint">
        <origin xyz="0 0 0.2" rpy="-${pi/2} 0 0"/>
        <child link="link_1_link"/>
        <parent link="arm_base_link"/>
        <axis xyz="0 0 1" rpy="0 0 0"/>
        <limit effort="100" velocity="100" lower="-${pi/2}" upper="${pi/2}"/>
        <dynamics damping="0.7" friction="1.0"/>
    </joint>

    <link name="link_1_link">
        <visual>
            <origin xyz="0 -${0.385/2} 0" rpy="${pi/2} 0 0"/>
            <geometry>
                <mesh filename="$(find custom_robot_sim)/meshes/link_1_mesh.stl" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <cylinder length="0.135" radius="${0.075/2}" />
            </geometry>
        </collision>
        <collision>
            <origin xyz="0 -${0.385/2} 0" rpy="${pi/2} 0 0"/>
            <geometry>
                <cylinder length="0.305" radius="${0.075/2}" />
            </geometry>
        </collision>
        <collision>
            <origin xyz="0 -${0.385} 0" rpy="0 0 0"/>
            <geometry>
                <cylinder length="0.135" radius="${0.075/2}" />
            </geometry>
        </collision>
        <xacro:inertial_cylinder radius="${0.075/2}" length="0.305" density="${density_pla}">
            <origin xyz="0 -${0.385/2} 0.0" rpy="${pi/2} 0 0"/>
        </xacro:inertial_cylinder>
    </link>
```
