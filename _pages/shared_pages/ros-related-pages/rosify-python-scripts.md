In this lecture, you will learn how to use custom Python libraries in your ROS package. We are going to *ROS-ify* some example Python scripts that you are already familiar with, which depends on the [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) and [adatools](https://github.com/frdedynamics/adatools/tree/master) libraries.


# How to import custom libraries

So far, you have been introduced to how to control Dynamixel robots using [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK), how to plot different features of the robot using [Robotics Toolbox](https://github.com/petercorke/robotics-toolbox-python). All are using libraries written in Python3.

On the other hand, recently, you have been introduced [ROS](https://docs.ros.org/en/foxy/Tutorials.html). You know that you can use Python or C++ to program nodes and launches in ROS. Today, we are going to learn how to "ROS-ify" simple Python scripts such that you can implement your Python scripts into the ROS environment.

Let's start by creating a new package. 

1. Open your favorite terminal: **Ctrl+Alt+T**
1. Make sure that you are in the right directory: `cd ~/ros2_ws/src`
1. Create a new package with a node: `ros2 pkg create --build-type ament_python --node-name send_single_joint_cmd rosify_dynamixel_ros_pkg`
1. Direct to the workspace back: `cd ..`
1. Compile the workspace: `colcon build --symlink-install`
1. Source the workspace: `source install/setup.bash`

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

To make things easier, I am going to create a custom Dynamixel class. Here is the dummy version just to remember how to create custom classes in Python:

~/adatools/custom_dxl/CustomDXL.py

```python
import dynamixel_sdk as dxl

class CustomDXL:
   def __init__(self,dxl_ids=[1, 2, 3]):
      self.dxl_ids=dxl_ids
   def getIDs(self):
      return self.dxl_ids

```

~/adatools/examples/test.py
```python
from custom_dxl.CustomDXL import CustomDXL

object_dxls = CustomDXL([60, 61])
print(f"IDs: {object_dxls.getIDs()}")
```

You can download the full version [here](https://github.com/frdedynamics/ros2_students_25.git) **rosify_dynamixel** folder. Remember to change the port, baud_rate and motor IDs.

{: .notice--info}
At this point, you should be able to control your motors if you copy-paste the content in [test_CustomDXL.py](https://github.com/frdedynamics/ros2_students_25/blob/master/rosify_dynamixel/test_CustomDXL.py) into the node `~/ros2_ws/src/rosify_dynamixel_ros_pkg/rosify_dynamixel_ros_pkg/send_single_joint_cmd.py`.


## Use CustomDXL in a ROS node
In [Publisher/Subscriber lesson](https://frdedynamics.github.io/hvl_robotics_website/courses/ada526/pub-sub#creating-ros-nodes), we have learned how a simple subscriber looks like. It will look like this when we add **CustomDXL** library into **send_single_joint_cmd.py**:

*~/ros2_ws/src/rosify_dynamixel_ros_pkg/rosify_dynamixel_ros_pkg/send_single_joint_command.py*
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from custom_dxl.CustomDXL import CustomDXL

#TODO
IMPORT NECESSARY MESSAGE LIBRARIES

class myDynamixelController(Node):
    def __init__(self) -> None:
        super().__init__("my_dynamixel_controller")
        #TODO
        self.sub = self.create_subscription(MSG_TYPE, 'TOPIC_NAME', self.listener_callback, 10)
        self.dxls = CustomDXL(dxl_ids=[60, 61]) ## TODO set your own dynamixel motor IDs
        self.dxls.open_port()
        self.dxls.send_goal(goal_pos=[1000, 2000]) ## TODO Adjust position commands according to your motors, give a list of all joints.
        print("Created")

    def listener_callback(self, msg):
        #TODO
        # listen to the message,
        # set it to the respective motor
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

    class myDynamixelController(Node):
        def __init__(self) -> None:
            super().__init__("my_dynamixel_controller")
            self.sub = self.create_subscription(Int32, '/dxl_joint_cmd', self.listener_callback, 10)
            self.dxls = CustomDXL()
            self.dxls.open_port()
            self.dxls.send_goal(goal_pos=[1000]) ## Random initial positions to all motors

            print("Created")
            print("Publish data between [0, 4095] to the topic /dxl_joint_cmd")

        def listener_callback(self, msg):
            print("Position command received.")
            self.dxls.send_single_goal(motor_order=0, goal_pos=[msg.data])
            
        
    def main(args=None):
        rclpy.init(args=args)
        node = myDynamixelController()
        rclpy.spin(node)

        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    </code></pre>
</div>


# Visualize

From [plot_base_plate](https://github.com/frdedynamics/adatools/blob/master/examples/plot_base_plate.py) example, we know how to visualize our custom robot. You can also rosify this.

{: .notice--info}
Create a new node called **visualize_dxl.py** in your package.

For those who already modified the [adatools.config_generator](https://github.com/frdedynamics/adatools/blob/master/adatools/config_generator.py) library, you can use your version and modify the following code. I have modified it such that created a new function for a 2-DOF model. Therefore, the **visualize_dxl.py** looks like this for me:

*~/ros2_ws/src/rosify_dynamixel_ros_pkg/rosify_dynamixel_ros_pkg/visualize_dxl.py*
```python
#!/usr/bin/env python3

from adatools import config_generator as cg
from adatools import plotting_tools as pt


my_conf2_robot = cg.get_robot_config_2(link1=0.3, link1_offset=0.0,
                                       link2=0.3, link2_offset=0.0)

# Plot the robot on base plate
robot_plot = my_conf2_robot.plot(my_conf2_robot.q, backend='pyplot')
pt.plot_baseplate(robot_plot)
# robot_plot.hold()

# Jog the robot on base plate
robot_teach = my_conf2_robot.teach(my_conf2_robot.q, backend='pyplot', block=False)
pt.plot_baseplate(robot_teach)
robot_teach.hold()
```

It is still a simple Python script, not a ROS node yet. Therefore, `ros2 run ...` does not work yet. You can test it with the following command: `python3 visualize_dxl.py` (run this command in the path where visualize_dxl.py is located). 

## Rosify visualize_dxl.py

First, add visualize_dxl.py in **setup.py**.

Second, modify the command such that it subscribes to the same topic as in the **send_single_joint_cmd.py**, which is `/dxl_joint_cmd`, and plots the value in this topic for the first joint.

```python
#!/usr/bin/env python3

from adatools import config_generator as cg
from adatools import plotting_tools as pt
from math import radians as d2r

#TODO: Add necessary ROS libraries


class myDynamixelVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("my_dynamixel_visualizer")
        #TODO: Create subscription
        self.create_config()
        self.create_visualizer()
        print("Created")

    def create_config(self):
        #TODO: Modify according to your robot
        self.my_conf_robot = cg.get_robot_config_2(link1=0.3, link1_offset=0.0,
                                        link2=0.3, link2_offset=0.0)
        
    def create_visualizer(self):
        self.robot_teach = self.my_conf_robot.teach(self.my_conf_robot.q, backend='pyplot', block=False)
        self.plot = pt.plot_baseplate(self.robot_teach)

    def listener_callback(self, msg):
        print("Received new data")
        #TODO: Assign the msg.data to the first joint of my_conf_robot.q
        #Hint: if you want to give degree commands, you need to convert it to radians. Use d2r()
        print(self.my_conf_robot.q)
        # Jog the robot on base plate
        self.plot.step()
    
def main(args=None):
    rclpy.init(args=args)
    node = myDynamixelVisualizer()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Our package now look like this:

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/ada526-rosify1.png)

Currently, we update the plot on *every* new data, even if the data is the same. It is quite inefficient. You can optimize such that you only update the plot when a different value is received. Brainstorm here on how to achieve it.



<button id="toggleButton">Solution</button>
<div id="hiddenText" style="display: none;">
    <pre><code class="python">

    #!/usr/bin/env python

    from adatools import config_generator as cg
    from adatools import plotting_tools as pt
    from math import radians as d2r

    #TODO: Add necessary ROS libraries
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Int32


    class myDynamixelVisualizer(Node):
        def __init__(self) -> None:
            super().__init__("my_dynamixel_visualizer")
            self.sub = self.create_subscription(Int32, '/dxl_joint_cmd', self.listener_callback, 10)
            self.create_config()
            self.create_visualizer()
            self.last_q = 0.0
            
            print("Created")

        def create_config(self):
            #TODO: Modify according to your robot
            self.my_conf_robot = cg.get_robot_config_2(link1=0.3, link1_offset=0.0,
                                            link2=0.3, link2_offset=0.0)
            
        def create_visualizer(self):
            self.robot_teach = self.my_conf_robot.teach(self.my_conf_robot.q, backend='pyplot', block=False)
            self.plot = pt.plot_baseplate(self.robot_teach)

        def listener_callback(self, msg):
            print("Received new data")
            ## Optimize:
            if d2r(int(msg.data)) == self.last_q:
                print("No update required")
            else:
                print("Plot updated")
                self.last_q = d2r(int(msg.data))
                self.my_conf_robot.q[0] = self.last_q
                print(self.my_conf_robot.q)
                # Jog the robot on base plate
                self.plot.step()
        
    def main(args=None):
        rclpy.init(args=args)
        node = myDynamixelVisualizer()
        rclpy.spin(node)

        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    </code></pre>
</div>

If the button doesn't work: [optimized_visualize_dxl.py](https://github.com/frdedynamics/ros2_students_25/blob/master/rosify_dynamixel_ros_pkg/optimized_visualize_dxl.py)

# Voluntary Exercise: Update your robot via visualizer

We have two nodes, one of which controls the real robot (send_single_joint_cmd.py) and the other visualizes the robot simulation. Often, you would like to see your robot configuration according to the given joint positions. Only after you are satisfied with the safety of the robot in this configuration, update the configuration of the real robot.

Let's do this.

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/ada526-rosify2.png)

- Modify the **send_single_joint_cmd.py** such that it subscribes to the topic `/dxl_actual_joint_cmd`.
- Create a new middleware node names **visualizer_to_robot.py**. It should subscribe to the same topic as the visualizer (`/dxl_joint_cmd`) does and publishes to the actual joint command topic (`dxl_actual_joint_cmd`) when a bool ROS parameter sets to `True`. The code for that looks like this: [visualizer_to_robot.py](https://github.com/frdedynamics/ros2_students_25/blob/master/rosify_dynamixel/visualizer_to_robot.py)
- Run all three nodes in the figure above. (or even better: make a launch file!)
- Either modify the existing visualizer node or create a new one [visualization_publisher.py](https://github.com/frdedynamics/ros2_students_25/blob/master/rosify_dynamixel_ros_pkg/visualization_publisher.py)
- Set update_robot parameter *True* whenever you want to send it to the robot: `ros2 param set /dynamixel_visualizer_to_controller update_robot True`

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/ros2-param-list-update-robot.png)
