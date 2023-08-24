Launch files are very common in ROS. They provide a convenient way to start up multiple nodes, as well as other initialization requirements such as setting parameters.

There are a few ways to create launch files in ROS2;  Python Launch Files, XML Launch Files, Composable Launch Files, Launch Configuration Files, Launch Configuration Files, etc. We will only learn how to create** Python launch files**.

<!-- [https://www.youtube.com/watch?v=dY9aZVMC-JM&ab_channel=HummingbirdRobotics](https://www.youtube.com/watch?v=dY9aZVMC-JM&ab_channel=HummingbirdRobotics)

or

[https://www.youtube.com/watch?v=xJ3WAs8GndA&ab_channel=RoboticsBack-End](https://www.youtube.com/watch?v=xJ3WAs8GndA&ab_channel=RoboticsBack-End) -->


## Create a launch file

1. Open a terminal: **Ctrl+Alt+T**
1. Change directory to your package: `cd ~/ros2_ws/src/my_package`
1. Create a launch folder: `mkdir launch`
1. Go to the newly creatged **launch** folder: `cd launch`
1. Create a launch file in it: `touch my_launch.launch.py`

Your directory should look like this:
![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/launch.png)

Now copy the code below and go through line-by-line.

*/my_package/launch/my_launch.launch.py*

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    hello_pub = Node(
        package="my_package",
        executable="my_publisher", # We defined in the setup.py
        name="my_publisher_from_node" # new name of the node if you want
    )

    hello_sub = Node(
        package="my_package",
        executable="my_subscriber",
        name="my_subscriber_from_node"
    )

    return LaunchDescription([hello_pub, hello_sub])
```


## Preparing the setup.py

We need to tell the ROS that we have a new executable. It is a bit different than introducing a node as executable (as we did in the previous tutorial). Here, we include the whole /launch folder.

1. Add the necessary imports `setuptools` and `glob`. 
1. Expand the **data_files[]** such that you add this line: `(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),`

*/my_package/setup.py*
```python
from setuptools import setup
import os ## Add-1
from glob import glob  ## Add-2

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))), ## Add-3
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gizem',
    maintainer_email='gizem@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_package = my_package.my_package:main',
            'my_publisher = my_package.my_publisher:main',
            'my_subscriber = my_package.my_subscriber:main'
        ],
    },
)
```

Done. Compile and source.

## Run the launch file

The command to run a launch file is very similar to running a node: `ros2 launch my_package my_launch.launch.py`

Now you can observe that two *hello world* nodes from the previous tutorial are running. (Hint: use either `ros2 node list` or `ros2 run rqt_graph rqt_graph`)

## Voluntary exercise

Can you modify the launch file such that it also starts the `turtlesim_node` and `turtle_teleop_key` nodes from the previous tutorial? The result should look like this:

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/launch2.png)

<button id="toggleButton">Click here to see the solution</button>
<div id="hiddenText" style="display: none;">
    <pre><code class="python">
    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        hello_pub = Node(
            package="my_package",
            executable="my_publisher", # We defined in the setup.py
            name="my_publisher_from_node" # new name of the node if you want
        )

        hello_sub = Node(
            package="my_package",
            executable="my_subscriber",
            name="my_subscriber_from_node"
        )

        turtle_sim = Node(
            package="turtlesim",
            executable="turtlesim_node",
            name="my_turtlesim_node"
        )

        teleop = Node(
            package="turtlesim",
            executable="turtle_teleop_key",
            name="my_teleop_node"
        )

        return LaunchDescription([hello_pub, hello_sub,
                                turtle_sim, teleop])

    </code></pre>
</div>

