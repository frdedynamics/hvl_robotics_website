In this tutorial, you will understant how ROS packages work and you will create your first ROS package.

## What is a Package?

A ROS package is simply a folder under your workspace. The name of the workspace can be various and you may want to create a new workspace for different purposes. For now, we will stick to what is readily available in our virtual machine and we will refer to this workspace as **~/ros2_ws** throughout the tutorials.

Although a ROS package is a simple folder, not every folder is a ROS package. For a folder to be a ROS package, it should have these properties:

1. It should be located under **~/ros2_ws**,
1. It should have **setup.py** file, **package.xml** file, and a **src** directory.

{: .notice--info}
Note that the second property is for Python-based ROS packages. It is possible (and very common) to create a C++ based package but we skip giving details about C++ packages in order not to make unnecessary confusion. You can read more about ROS packages in this [link](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html).


## Create Your First Package
`ros2 pkg create --build-type ament_python --node-name my_package my_package`

In the above command, you can replace **my_package** with the desired name for your package but we will stick to that for this tutorial.. This command create a new ROS 2 Python package with the specified name.

After running the command, you will see a new directory named my_package created in your current working directory. Inside this  directory, you will find the basic structure of a ROS 2 Python package, including the **setup.py** file, **package.xml** file, and a **src** directory.
You can then navigate into the **~/ros2_ws/src/my_package/** directory and start developing your Python code for the package. You can create Python scripts, modules, or any other necessary files within this directory to implement the functionality of your package.

## Source Your Workspace
`source ~ros2_ws/install/setup.bash`

or if you are already in **~/ros2_ws** directory in your terminal:

`source install/setup.bash`

*Sourcing* is quite a Linux process, actually. It tells your system "run the commands in this file". You source a **.bash** (or sometimes **.sh**) file which is a file that contain a sequence of commands that are executed by the bash program line by line. It is mentioned in more detail in our [Linux Tutorial](https://frdedynamics.github.io/hvl_robotics_website/linux) page.

The reason is that, after you run `colcon build`, you compile all the source code under your workspace, and do little or much changes in your ROS system. To tell the Linux system what you have changed in the *ROS side*, you source the necessary **.bash** (or **.sh**) file in your terminal.

Therefore you HAVE TO source your **setup.bash** after *every time* you create a package or a file in your workspace and *most of the time* you do changes on an existing file. We will talk about when it is relevant later.

<!-- https://www.youtube.com/watch?v=Gg25GfA456o&ab_channel=RoboticsBack-End -->


<!-- {% include video id="Y-0OyxoN4xU" provider="youtube" %} -->