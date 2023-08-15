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


### Turtlesim Tutorial

In this part of the tutorial, we will learn about a very common topic `/cmd_vel` which often controls the velocity of a robot.

As we mentioned before, ROS has a sweet obsession with turtles. The logos of each ROS distribution has a turtle, the mobile robots which you will work on lab assignments are called *Turtlebot*s and the tutorial that we will do now is on *turtlesim*.

**Run the turtlesim node:** `ros2 run turtlesim turtlesim_node`. You will see a simulated turtle.

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/turtlesim-node.png)

{: .notice--info}
The `turtlesim` is a package that comes with ROS generic installation. You do not see a package named *turtlesim* under your `~/ros2_ws/src` directory but the code above works just fine! 
If you are curious, all the default packages are in */opt/ros/foxy/share*.

In the ROS world, we can say that this turtle represents a mobile robot. We can control it as if it was a robot then. Luckily, the `turtlesim` package has an implemented *publisher node* that publishes `\cmd_vel` topic to the `turtlesim_node`.

**Run the teleoperation node:** `ros2 run turtlesim turtle_teleop_key`. You will be able to control the turtle with the arrow keys on your keyboard.

{: .notice--danger}
Make sure that the terminal which the `turtle_teleop_key` node is running is selected, NOT THE SIMULATION WINDOW. Otherwise, you cannot control the turtle.

### Visualize Nodes and Topics with rqt

At the moment, a lot is going on in the background. 

1. There are 2 nodes running: `ros2 node list`
    ```
    /teleop_turtle
    /turtlesim
    ```
1. A few topics are available: `ros2 topic list` - one of which is **/cmd_vel**.
    ```
    /parameter_events
    /rosout
    /turtle1/cmd_vel
    /turtle1/color_sensor
    /turtle1/pose
    ```
1. The message type of the **/turtle1/cmd_vel** is *Twist*: `ros2 topic info /turtle1/cmd_vel`. There are 1 publisher and 1 subscriber node of this topic.

And there is much more that you can observe with `ros2 topic/param/service/node list/info` but these are enough for this tutorial. You will learn different [communication patterns of ROS](https://frdedynamics.github.io/hvl_robotics_website/courses/dat160/ros-comm-pattern) later. These will make more sense there.

One last cool thing is that you can see all these visually also. 

**Type:** `ros2 run rqt_graph rqt_graph`

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/rqt-graph.png)

### Turtlesim and /cmd_vel Exercise

This part is voluntary. 

Can you write a publisher that makes the turtle draw a circle?

<button id="toggleButton">Click here to see the solution</button>
<div id="hiddenText" style="display: none;">
    <pre><code class="python">
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
    </code></pre>
</div>
