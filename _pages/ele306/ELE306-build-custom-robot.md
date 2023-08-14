---
layout: single
title: "Build a Custom Robot"
permalink: /courses/ele306/build-custom-robot
toc: true # The right side bar "on this page"
breadcrumbs: true  # the directory-of-documents type of header navigation
sidebar:
  nav: "ele306"  # the left navigation bar. Choose which category you want.
taxonomy: markup
# {% include_absolute '_pages/shared_pages/ros-related-pages/build-custom-robot.md' %}
---

## Basic Xacro Robot Description structure
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

## Links

### Basic Structure

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
    <interial>
        <!-- ADD INERTIAL PARAMETERS HERE -->
    </interial>
</link>
```

### Geometries

#### Box

```xml
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

### Continuous Joint
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

