---
layout: single
title: "Namespace"
permalink: /courses/ada526/namespace
toc: true # The right side bar "on this page"
breadcrumbs: true  # the directory-of-documents type of header navigation
sidebar:
  nav: "ada526"  # the left navigation bar. Choose which category you want.
taxonomy: markup

my_variable: scripts.html
---

In ROS, Namespaces are used to allow multiple topics with the same name to exists. It works similar to a folder structure in any operating system where inside one folder no duplicate names are allowed but in a different folder a file can have the exact same name. We therefore use Namespaces to have multiple topics with the same name e. g. when launching multiple Turtlebots we need two different “cmd_vel” topics to control each of the robots.

Inside a launch file you can use the following code to launch a node inside a namepsace. **Don’t forget to replace the in uppercase written names.**
```python
def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='PACKAGE_NAME',
            executable='EXECUTABLE_NAME',
            namespace='NAMESPACE_NAME',
            name='NODE_NAME'),
    ])
```

**When referencing topic names inside a Namepspace be aware of the syntax.** Defining the name with a leading “/” means that I will define the full name with all Namespaces e.g. “/tb3_0/odom”. If you don’t put the leading “/” you are defining the name from the Namespace you are in. If I launch my ROS Node inside the Namespace “tb3_0” I can then reference the same topic as before with just “odom”.