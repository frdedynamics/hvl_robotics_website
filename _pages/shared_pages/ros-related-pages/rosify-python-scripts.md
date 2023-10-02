So far, you have been introduced how to control Dynamixel robots using [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK), how to plot different features of the robot using [Robotics Toolbox](https://github.com/petercorke/robotics-toolbox-python). All are using libraries written in Python3.

On the other hand, recently, you have been introduced [ROS](https://docs.ros.org/en/foxy/Tutorials.html). You know that you can use Python or C++ to program nodes and launches in ROS. Today, we are going to learn how to "ROS-ify" simple Python scripts such that you can implement your Python scripts into the ROS environment.

Let's start by creating a new package. 

1. Open your favorite terminal: **Ctrl+Alt+T**
1. Make sure that you are in the right directory: `cd ~/ros2_ws/src`
1. Create a new package with a node: `ros2 pkg create --build-type ament_python --node-name send_single_joint_cmd rosify_dynamixel_ros_pkg`
1. Direct to the workspace back: `cd ..`
1. Compile the workspace: `colcon build --symlink-install`
1. Source the workspace: `source install/setup.bash`

# How to import custom libraries

The ROS packages *have to be* under your ROS workspace. It is `~/ros2_ws` in the virtual machine. This might cause an issue in the beginning for some manually installed Python packages that you want to use (i.e adatools). First, you need to import the Python path of the sensors, motors, visualizer scripts etc. libraries into your `$PYTHONPATH`. Otherwise, you will get `ImportError` or `ModuleNotFoundError` when you attempt to use those libraries.

Check your Python libraries path: `echo $PYTHONPATH`

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/echo-pythonpath.png)

You can do in manually every time, or package specify, but we will focus on a global solution since you may want to skip this step in future occasions.

## Import adatools path

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

# Exercise: ROSify a Dynamixel script

Let's ROSify a Dynamixel script. Our end goal at this step is to make a ROS node that subscribes to a simple user command in terms of a single joint command topic and controls the respective Dynamixel motor.

## CustomDXL class

To make things easier, I am going to create a custom Dynamixel class. Here is the dummy version just to remember creating custom classes in Python:

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

object_dxls = CustomDXL(5)
print(f"Val: {object_dxls.getVal()}")
```

You can download the full version [here](https://github.com/frdedynamics/ros2_students.git) **rosify_dynamixel** folder. Remember to change the port, baud_rate and motor IDs.

{: .notice--info}
At this point, you should be able to control your motors if you copy-paste the content in [test_CustomDXL.py](https://github.com/frdedynamics/ros2_students/blob/master/rosify_dynamixel/test_CustomDXL.py) into the node `~/ros2_ws/src/rosify_dynamixel_ros_pkg/rosify_dynamixel_ros_pkg/send_single_joint_cmd.py`.


## Use CustomDXL in a ROS node
In [Publisher/Subscriber lesson](https://frdedynamics.github.io/hvl_robotics_website/courses/ada526/pub-sub#creating-ros-nodes), we have learned how a simple subscriber looks like. It will look like this when we add **CustomDXL** library:


```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from custom_dxl.CustomDXL import CustomDXL

IMPORT NECESSARY MESSAGE LIBRARIES

class myDynamixelController(Node):
    def __init__(self) -> None:
        super().__init__("my_dynamixel_controller")
        self.sub = self.create_subscription(MSG_TYPE, 'TOPIC_NAME', self.listener_callback, 10)
        self.dxls = CustomDXL()
        self.dxls.open_port()
        self.dxls.send_goal_all_joints(goal=[1000, 2665]) ## Random initial positions to all motors
        self.dxls.read_pos()
        print("Created")

    def listener_callback(self, msg):
        pass
    
def main(args=None):
    rclpy.init(args=args)
    node = myDynamixelController()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```


{: .notice--info}

Remember different message types. This is something you need to choose based on the data that you want to provide to ROS in terms of sensor publisher or that you get from ROS in terms of actuator commands. We will use [**std_msgs/Int32 Message**](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Int32.html) in this exercise.


**Your turn:**

Imagine a topic `/dxl_joint_cmd` (Int32) is supposed control the first joint of your robot. Can you modify the code such that you will subscribe to this topic and actuate your motor accordingly? 

Hint:

1. Use `rqt_publisher` to publish `/dxl_joint_cmd`.
2. Update joint value every second.


<button id="toggleButton">Click here to see the solution</button>
<div id="hiddenText" style="display: none;">
    <pre><code class="python">
    #!/usr/bin/env python3

    import rclpy
    from rclpy.node import Node

    from custom_dxl.CustomDXL import CustomDXL

    from std_msgs.msg import Int32
    import time
    import sys

    class myDynamixelController(Node):
        def __init__(self) -> None:
            super().__init__("my_dynamixel_controller")
            self.sub = self.create_subscription(Int32, '/dxl_joint_cmd', self.listener_callback, 10)
            self.dxls = CustomDXL()
            self.dxls.open_port()
            self.dxls.send_goal_all_joints(goal=[1000, 2665]) ## Random initial positions to all motors
            self.dxls.read_pos()

            print("Created")

        def listener_callback(self, msg):
            print("here")
            self.dxls.send_goal_single_joint(0,int(msg.data))
            self.dxls.read_pos()
        
    def main(args=None):
        rclpy.init(args=args)
        node = myDynamixelController()
        rclpy.spin(node)

        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    </code></pre>
</div>

