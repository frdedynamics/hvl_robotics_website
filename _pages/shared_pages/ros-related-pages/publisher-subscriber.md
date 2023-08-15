In this tutorial, you will learn ROS Nodes (as publisher and subscriber), and create your first two nodes which communicate with each other.

## How does publisher/subscriber work?

![image-center](https://docs.ros.org/en/humble/_images/Nodes-TopicandService.gif)

Nodes are the simplest executable files of a ROS package. They are either written in Python or C++.

In the ROS framework, there are various ways that nodes communicate with each other such as via *topic*, *request/response* or *parameter*. All have advantages and disadvantages but we will focus on *topic*s in this tutorial.

## Creating ROS Nodes
A ROS node can publish a topic, subscribe to a topic or can to both with several topics. We just need to define it in the code. A regular ROS node (as a publisher) would look like this:

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/node-overview.png)

Maybe this is too overwhelming for the start. Let's go step by step.

### Create Publisher

Create a Python script in the package: 

`touch ~/ros2_ws/src/my_package/my_package/my_publisher.py`

#### Simple Python Script

This is a simple Python script.

*ros2_ws/src/my_package/my_publisher.py*

```python
def main():
    print('Hi from my_package.')

if __name__ == '__main__':
    main()
```

#### Simple Publisher
This is a simple publisher node that does nothing:

*ros2_ws/src/my_package/my_publisher.py*

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

We need to add an entry point in `setup.py`. This will tell the ROS what to run as an executable node.

```python
entry_points={
        'console_scripts': [
            'my_publisher = my_package.my_publisher:main'
        ],
    },
```

and then compile: `colcon build` and source `source install/setup.bash`

{: .notice--info}

Note that 1) File name of the node, 2) Node name in the code, and 3) Executable name in the `setup.py` are not necessarily the same. Nonetheless, it is easier to follow if we keep all the same for now.

#### Create a Timer in the Publisher

Timers are the functions that are called periodically. If we want to publish something, let's say every second, we need to use a timer.

**What to do:** Create a timer function, call it in the init() and spin in the main()

*ros2_ws/src/my_package/my_publisher.py*

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

What does this node do? Can you explain? Is there anything missing?

Yes,

This piece of code is a ROS node, pretty much done, but it only *prints* "Hello" but not publishes. As the last step of creating a publisher, we will publish the text "Hello" and publish it as a *String message". Here is the complete code:

*ros2_ws/src/my_package/my_publisher.py* (Completed)

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class myPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("my_publisher")
        self.pub = self.create_publisher(String, 'topic', 10)
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.pub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = myPublisherNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Create Subscriber

We have a node publishing the string `"Hello`"` at the moment. To make it more meaningful, we can create another node that listens to this string. We call these types of nodes **Subscriber**s. Let's copy-paste the code piece below and discuss how it works.

1. Create the Python script: `touch ~/ros2_ws/src/my_package/my_package/my_subscriber.py`

2. Copy-paste the code below:

    *ros2_ws/src/my_package/my_subscriber.py* (Completed)
    ```python
    #!/usr/bin/env python3

    import rclpy
    from rclpy.node import Node

    from std_msgs.msg import String

    class mySubscriberNode(Node):
        def __init__(self) -> None:
            super().__init__("my_subscriber")
            self.sub = self.create_subscription(String, 'topic', self.listener_callback, 10)
            print("Created")

        def listener_callback(self, msg):
            self.get_logger().info('I heard: "%s"' % msg.data)

    def main(args=None):
        rclpy.init(args=args)
        node = mySubscriberNode()
        rclpy.spin(node)

        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

3. Add an entry point in `setup.py`.
    ```python
    entry_points={
            'console_scripts': [
                'my_publisher = my_package.my_publisher:main',
                'my_subscriber = my_package.my_subscriber:main'
            ],
        },
    ```

### Run nodes

Now we have a publisher and a subscriber nodes. It is time to run them and observe that they communicate successfully.

1. Open your favorite terminal: **Ctrl+Alt+T**
1. Make sure that you are in the right directory: `cd ~/ros2_ws`
1. Compile the workspace: `colcon build --symlink-install`
1. Source the workspace: `source install/setup.bash`
1. Run publisher: `ros2 run my_package my_publisher`
1. Open a new terminal:**Ctrl+Alt+T**
1. Run subscriber: `ros2 run my_package my_subsciber`


## Understanting Topics/Messages

A **message is data** and a **topic is the channel** where nodes are subscribed to read messages or where the nodes publish those messages.

So far, we only focused on the *String* type of message and wrote a simple text. There are other message types which are used commonly in robotics projects such as *Pose*, *Position*, *Vector3*, *Twist*, etc. Each message type in ROS is defined in the respective library. For instance, the *String* is in the standard messages library as you see in the beginning of the previous codes: `from std_msgs.msg import String`.

The message types can be quite generic like in [geometry_msgs](https://docs.ros2.org/latest/api/geometry_msgs/index-msg.html) and [std_msgs](https://docs.ros2.org/latest/api/std_msgs/index-msg.html), or intended to be used in specific cases like in [sensor_msgs](https://docs.ros2.org/latest/api/sensor_msgs/index-msg.html) and [nav_msgs](https://docs.ros2.org/latest/api/nav_msgs/index-msg.html). You can also create your own message type, which will be discussed later.


### Available Topics of *turtlesim*

In this part of the tutorial, we will learn about a very common topic `/cmd_vel` which often controls the velocity of a robot.

As we mentioned before, ROS has a sweet obsession with turtles. The logos of each ROS distribution has a turtle, the mobile robots which you will work on lab assignments are called *Turtlebot*s and the tutorial that we will do now is on *turtlesim*.

Run the turtlesim node: `ros2 run turtlesim turtlesim_node`

The `turtlesim` is a package that comes with ROS generic installation. You do not see a package named *turtlesim* under your `~/ros2_ws/src` directory but the code above works just fine! 

{: .notice--info}
If you are curious, all the default packages are in */opt/ros/foxy/share*.

You can v

### Visualize Nodes and Topics with rqt

At the moment, there are a lot going on in the backg




## Simple Publisher

Make turtlesim draw circle.

*/my_package/src/my_draw_circle.py*

```python
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
```

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