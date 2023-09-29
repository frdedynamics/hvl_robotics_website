So far, you have been introduced how to control Dynamixel robots using [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK), how to plot different features of the robot using [Robotics Toolbox](https://github.com/petercorke/robotics-toolbox-python). All are using libraries written in Python3.

On the other hand, recently, you have been introduced [ROS](https://docs.ros.org/en/foxy/Tutorials.html). You know that you can use Python or C++ to program nodes and launches in ROS. Today, we are going to learn how to "ROS-ify" simple Python scripts such that you can implement your Python scripts into the ROS environment.

# Remember ROS publisher steps
In [Publisher/Subscriber lesson](https://frdedynamics.github.io/hvl_robotics_website/courses/ada526/pub-sub#creating-ros-nodes), we have learned how a simple publisher looks like. Remember the steps:

## Simple Publisher

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class myPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("my_publisher")
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hello")

def main(args=None):
    rclpy.init(args=args)
    node = myPublisherNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

{: .notice--info}

## Message types

Remember different message types. This is something you need to choose based on the data that you want to provide to ROS in terms of sensor publisher or that you get from ROS in terms of actuator commands. Some relevant message types are given below.

[**geometry_msgs/Twist Message**](https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html)
* Vector3  linear
    * float64 x
    * float64 y
    * float64 z
* Vector3  angular
    * float64 x
    * float64 y
    * float64 z

[**std_msgs/Float32 Message**](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float32.html)
* float32 data

[**std_msgs/Float64MultiArray Message**](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64MultiArray.html)
* std_msgs/MultiArrayLayout layout
  * std_msgs/MultiArrayDimension[] dim
    - string label 
    - uint32 size 
    - uint32 stride 
  * uint32 data_offset
* float64[] data

# Import necessary libraries

The ROS packages *have to be* under your ROS workspace. It is `~/ros2_ws` in the virtual machine. This might cause an issue in the beginning for some manually installed Python packages that you want to use (i.e adatools). First, you need to import the Python path of the sensors, motors, visualizer scripts etc. libraries into your `$PYTHONPATH`. Otherwise, you will get `ImportError` or `ModuleNotFoundError` when you attempt to use those libraries.

Check your Python libraries path: `echo $PYTHONPATH`

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/echo-pythonpath.png)

You can do in manually every time, or package specify, but we will focus on a global solution since you may want to skip this step in future occasions. 

Check the path of your desired library, for example **adatools**. Browse to the folder location, open a new terminal, type `pwd`, note the directory. For my case:

```
>pwd
/home/rocotics/adatools
```

Add this line to your `~/.bashrc` using your favorite text editor. >>> *export PYTHONPATH="${PYTHONPATH}:/my/other/path"*

For my case: `export PYTHONPATH="${PYTHONPATH}:/home/rocotics/adatools"`

Open a new terminal, or source .bashrc in your current terminal: `source ~/.bashrc`.

Check your Python libraries path again: `echo $PYTHONPATH`.

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/echo-pythonpath2.png)

You should do this to every single library that is installed from source.

## Sanity check

Create a dummy Python script in your ROS package and import your desired library:

```python
import sys
#sys.path.append('/home/rocotics/adatools') # this is to add manually instead of export in .bashrc

from adatools import utils

print(utils.__file__)
```


## Exercise: ROSify a Dynamixel script

Let's ROSify a Dynamixel script. To make things easier, I am going to create a custom Dynamixel class. 

/home/rocotics/adatools/custom_dxl/CustomDXL.py

```python
import dynamixel_sdk as dxl
import numpy as np
from adatools import utils
from sys import exit


class CustomDXL:
   def __init__(self,val):
      self.val=val
   def getVal(self):
      return self.val*self.val

```

/home/rocotics/adatools/examples/test.py
```python
from custom_dxl.CustomDXL import CustomDXL
# creating object for Square class
object1 = CustomDXL(5)
print(f"Val: {object1.getVal()}")
```