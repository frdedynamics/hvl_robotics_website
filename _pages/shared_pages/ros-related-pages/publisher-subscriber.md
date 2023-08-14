In this tutorial, you will learn ROS Nodes (as publisher and subscriber), and create your first two nodes which communicate with each other.

## How does publisher/subscriber work?

![image-center](https://docs.ros.org/en/humble/_images/Nodes-TopicandService.gif)

Nodes are the simplest executable files of a ROS package. They are either written in Python or C++.

In the ROS framework, there are various ways that nodes communicate with each other such as via *topic*, *request/response* or *parameter*. All have advantages and disadvantages but we will focus on *topic*s in this tutorial.

## ROS Node
A ROS node can publish a topic, subscribe to a topic or can to both with several topics. We just need to define it in the code. A regular ROS node (as a publisher) would look like this:

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/node-overview.png)

Maybe this is too overwhelming for the start. Let's go step by step.

### Create Publisher

Create a Python script in the package: 

`touch ~/ros2_ws/src/my_package/my_package/my_publisher.py`

#### Simple Python Script

This is a simple Python script.

```python
def main():
    print('Hi from my_package.')

if __name__ == '__main__':
    main()
```

#### Simple Publisher
This is a simple publisher node that does nothing:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class myPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("my_publisher")

def main(args=None):
    rclpy.init(args=args)
    node = myPublisherNode()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

At the moment, it works as a regular Python script but not as a ROS node. There is no autocomplete and `ros2 run` does not work.

We need to add entry point in `setup.py`:

```python
entry_points={
        'console_scripts': [
            'my_node = my_package.my_publisher:main'
        ],
    },
)
```

and then compile: `colcon build` and source `source install/setup.bash`

{: .notice--info}

Note that 1)File name of the node, 2) Node name in the code, and 3) Executable name in the `setup.py` are not necessarily the same. Nonetheless, it is easier to follow if we keep all the same for now.

#### Create Timer in the Publisher

Timers are the functions that are called periodically. If we want to publish something, let's say every second, we need to use a timer.

**What to do:** Create a timer function, call it in the init() and spin in the main()

*/my_package/my_publisher.py*

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

## Simple Publisher

Make turtlesim draw circle.

*/my_package/src/my_draw_circle.py*

    #!/usr/bin/env python3
    
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    
    class DrawCircleNode(Node):
        def __init__(self):
            super().__init__("draw_circle")
            self.get_logger().info("DrawCircleNode created")
            self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
            self.timer = self.create_timer(0.5, self.set_cmd_vel)
    
        def set_cmd_vel(self):
            msg = Twist()
            msg.angular.z = 1.0
            msg.linear.x = 1.0
            self.cmd_vel_pub.publish(msg)
    
    def main(args=None):
        rclpy.init(args=args)
        pub_node = DrawCircleNode()
        rclpy.spin(pub_node)
        rclpy.shutdown()
    
    if __name__ == '__main__':
        main()

**********Optional:********** add `geometry_msgs` and `turtlesim` in the `package.xml`.

********Needed:******** Add `draw_circle.py` in `setup.py`.

## Simple Subscriber

Log the position of the turtlesim:

*/my_package/src/my_pose_logger.py*

    #!/usr/bin/env python3
    
    import rclpy
    from rclpy.node import Node
    from turtlesim.msg import Pose
    from geometry_msgs.msg import Twist
    
    class PoseLoggerNode(Node):
        def __init__(self):
            super().__init__("pose_logger")
            self.pose_subscriber = self.create_subscription(
                Pose, "/turtle1/pose", self.pose_callback, 10)
            self.get_logger().info("Pose logger created")
    
        def pose_callback(self, msg: Pose):
            self.get_logger().info(str(msg))
    
    def main(args=None):
        rclpy.init(args=args)
        sub_node = PoseLoggerNode()
        rclpy.spin(sub_node)
        rclpy.shutdown()
    
    if __name__ == '__main__':
        main()

* It is nice to check out the *leaf topics* in `rqt_graph` to see unpublished/unsubscribed topics also.

## Finally the setup.py:

    from setuptools import setup
    
    package_name = 'my_package'
    
    setup(
        name=package_name,
        version='0.0.0',
        packages=[package_name],
        data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
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
                'my_node = my_package.src.my_node:main',
                'my_publisher = my_package.src.my_draw_circle:main',
                'my_subscriber = my_package.src.my_pose_logger:main'
            ],
        },
    )

```sequence
Alice->Bob: Hello Bob, how are you?
Note right of Bob: Bob thinks
Bob-->Alice: I am good thanks!
```