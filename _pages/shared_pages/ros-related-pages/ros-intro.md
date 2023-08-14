Welcome to the ROS section of the course. The purpose of these pages are to give you a compact ROS introduction where you can use in your course project.

## What is ROS?

This section serves as a look-up page where you can find processes and commands easily.

### Robot Operating System

(Robot Operating System) is an open-source,meta-operating system for your robot. It provides the services you would expect from an operating system, including hardware abstraction, low-level device control, implementation of commonly used functionality, message-passing between processes, and package management. It also provides tools and libraries for obtaining, building, writing, and running code across multiple computers.

![Number of ROS Users](https://global.discourse-cdn.com/business7/uploads/ros/original/2X/2/2e99e36f60b1ca9aee851f7f33ef1b573d26aaf4.png)

[2022 report](http://download.ros.org/downloads/metrics/metrics-report-2022-07.pdf)

### ROS versions

Our version is **Foxy Fitzroy** which is a ROS2 distribution released on the 5th of June 2020.

![image-center](https://docs.ros.org/en/foxy/_static/foxy-small.png)

And you can see the version history of ROS (both ROS1 and ROS2) and enjoy the nerdiness on alphabetic version naming and the sweet obsession of ROS on turtles.

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/rosVersions.jpg)
  
More updated versions of the history can be found on the [Wikipedia](https://en.wikipedia.org/wiki/Robot_Operating_System) page](https://en.wikipedia.org/wiki/Robot_Operating_System).

### Useful Resources

The documentation is prepared as a target-based concise summary of many cool features of ROS. However, a deeper and integrated understanding of ROS, we highly suggest you check out these tutorials:

* [Official ROS Documentation](https://docs.ros.org/en/foxy/)
* [TheConstructSim](https://www.theconstructsim.com/robotigniteacademy_learnros/ros-courses-library/)
* [A Concise Introduction to Robot Programming with ROS2](https://github.com/fmrico/book_ros2)

and some forums where you can ask questions:

* [Stackexchange for ROS](https://robotics.stackexchange.com/)
* [ROS Q&A](https://answers.ros.org/questions/)

{: .notice--info}

**IMPORTANT:** You should specify the ROS distribution (Foxy in our case) when you are using other sources and forums. There are several ROS versions and some of which are not compatible with each other. The ROS1 distributions (..., Kinetic, Melodic, Noetic) are somewhat compatible with each other. Also ROS2 distributions (..., Dashing, Eloquent, **Foxy**, Galactic, Humble, Iron) are somewhat compatible with each other. If you cannot find what you are looking for, you may want to check answers for other distributions but always keep in mind that there might be incompatibility between ROS2 distributions and ROS2 is *a lot different* than ROS1.

## Shortcuts, Commands, Definitions

For a beginner, both the Linux system and ROS structure can be confusing. To make this transition as smooth as possible for you, we prepared a Python tutorial and a Linux tutorial page. Please check these out if you have struggles in any of these subjects.

This section gives you an overview of the most used shortcuts, commands and definitions of the most used terms within ROS. Use this section as a cheat sheet.

### Shortcuts

* Open a new terminal: `Ctrl+Alt+T`
* Copy Paste: `Ctrl+Shit+C` and `Ctrl+Shit+V` (regular Ctrl+C andCtrl+V does not work in terminals)
* `Tab` for auto-complete
* Recursive search `Ctrl+R`

{: .notice--info}

**Terminal** and **IDE** are two different software. A **terminal** is a *window* that you can run commands. You may look like a hacker if you type things on a terminal in front of your parents.
Here is a screenshot of Terminator terminal (or you can use the generic Ubuntu terminal).
![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/terminal.png)
On the other hand, an **IDE** is a simple text editor. VSCode, Notepad, Atom or even Microsoft Word (LibreOffice for Ubuntu) can be considered as IDE. We write code in IDE -- not in a terminal (99% of the time).
Here is a screenshot of VScode.
![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/vscode.png)

### Commands

* `ros2 run my_pkg my_node`: Start a node (a .py or .cpp file)
* `ros2 launch my_pkg my_launch.launch`: Start a launch file
* `colcon build`: Compile colcon workspace
* `colcon build --symlink-install`: Compile colcon workspace such that you don't need to compile every time you change something in Python files.
* `source ~/ros2_ws/install/setup.bash`: loads the compiled shell environment (use after *almost* every `colcon build`)
* `rosdep install --from-paths src --ignore-src --rosdistro foxy -y`: Install dependencies of the packages in src. You need to be in the src folder and you may need `rosdep init` and/or `rosdep update` beforehand.
* `ros2 topic echo/list/info (topic_name)`: Listen/list/get information about available topics
* `ros2 param list/load/get/set`: Commands for parameter server
* `ros2 node list/info (topic_name)`: List/get information about available nodes

At any time, if you need help with a specific command, you can add -h to see a help message: `ros2 run -h`, `ros2 node -h`, `ros2 node list -h`, etc.

### Dictionary

![image-center](https://docs.ros.org/en/humble/_images/Nodes-TopicandService.gif)

* **node**: ROS executable
* **launch**: Multiple ROS executables as well as parameters andarguments
* **publisher**: An executable providing data to the ROS system
* **subscriber**: An executable retrieving data from the ROS system
* **package**: A folder contains several nodes/launchers/resourcedeveloped for a specific purpose.
* **topic**: The channel between (a) publisher and (a) subscriber(more like a sophisticated data type definition)
* **message**: The information in a topic
* **gazebo**: A powerful simulator with physical properties (gravity,collision, lights/shadows etc)
* **rviz**: A visualization software without physical properties
* **rqt**: Many useful ROS-Qt packages like rqt_graph, rqt_publisher,rqt_controller_manager, rqt_plot etc.
* **urdf/sdf**: File formats to define a robot
* **parameter server**: A shared dictionary of parameters that nodesstore and retrieve at runtime
* **tf**: Stands for *Transform* and it is a powerful ROS packagebuilds relationships of multiple frames from a given robot model.