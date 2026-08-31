In this tutorial, you will learn ROS Nodes (as publisher and subscriber), and create your first two nodes which communicate with each other.

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/ros-filesystem-pub-sub.png)

## How does publisher/subscriber work?

![image-center](https://docs.ros.org/en/jazzy/_images/Topic-MultiplePublisherandMultipleSubscriber.gif)

Nodes are the simplest executable files of a ROS package. They are either written in Python or C++.

In the ROS framework, there are various ways that nodes communicate with each other such as via *topic*, *request/response* or *parameter*. All have advantages and disadvantages but we will focus on *topic*s in this tutorial.

## Creating ROS nodes
A ROS node can publish a topic, subscribe to a topic or can to both with several topics. We just need to define it in the code.

### Simple Python script

This is a simple Python script.

```python
import a-fancy-library

class myFancyClass():
    # This is what automatically runs when you create an object from this class
    def __init__(self):
        print("initialized")

    # This you can call anywhere after you create the object
    def another_method(self):
        print('Hi from my method')

def main():
    print("Do something nice here.")
    my_object = MyFancyClass() # Output: initialized
    my_object.another_method() # Output: Hi from my method

if __name__ == '__main__':
    main()
```

So now we would like to add ROS elements to this simple code structure to make it a publisher and/or subscriber.


<!-- A regular ROS node (as a publisher) would look like this:

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/node-overview.png)

Maybe this is too overwhelming for the start. Let's go step by step. -->

### Create a publisher

Create a Python script in the package: 

`touch ~/ros2_ws/src/my_package/my_package/my_publisher.py`

#### Simple publisher
This is a simple publisher has five main things on top of this simple Python script:

1. Proper ros-python imports:
    ```python
    # Those two are a-must
    import rclpy
    from rclpy.node import Node

    # Depending on what topics you want to publish/subscribe, more libraries are needed
    from std_msgs.msg import String
    ```

2. A class from the ROS Node parent class:
    ```python
    class myPublisherNode(Node):
        def __init__(self) -> None:
            super().__init__("my_publisher") # This is what your node name would be
            self.pub = self.create_publisher(String, 'my_topic', 10) # This is what you's publish
            self.create_timer(1.0, self.timer_callback) # How often to publish - this publishes every second 
    ```

3. A method to do something periodically (*Psst: we have already connected this method to a timer in the __init__ function in the last line*):
    ```python
        def timer_callback(self): # or you can rename the method - it doesn't need to be called "timer"
            msg = String() # A msg type
            msg.data = 'Hello World' # Fill all the fields of the msg with meaningful info
            self.pub.publish(msg) # Publish it
            self.get_logger().info('Publishing: "%s"' % msg.data) # Optionally print things out in the terminal
    ```

4. A main function to gather all the tasks to be done in this publisher:
    ```python
    def main(args=None):
        rclpy.init(args=args)
        node = myPublisherNode()
        rclpy.spin(node)

        rclpy.shutdown()
    ```

5. And finally *boilerplate script execution check* or *main guard*, which is a fancy technical term referring a special conditional block for controlling code execution based on how a Python file is invoked. If you run this file as a regular Python script (using the file name like `python my_publisher.py`), the code here would run. However, if you import this as `from my_publisher import myPublisherNode`, it doesn't run this conditional block. It is very common thing in Python and particularly useful in ROS if you want to test your node without actually calling in your overall system.
    ```python
    if __name__ == '__main__':
        main()    
    ```

That's it! Let's gather everything in one Python script. You can just copy-paste the code below.

*ros2_ws/src/my_package/my_publisher.py*

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class myPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("my_publisher")
        self.pub = self.create_publisher(String, 'my_topic', 10)
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

#### Updating the package with the new publisher
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


### Create a subscriber

We have a node publishing the string `"Hello`"` at the moment. To make it more meaningful, we can create another node that listens to this string. We call these types of nodes **Subscriber**s. Let's copy-paste the code piece below and discuss how it works.

#### Simple subscriber

1.Create the Python script: `touch ~/ros2_ws/src/my_package/my_package/my_subscriber.py`

2.Copy-paste the code below:

*ros2_ws/src/my_package/my_subscriber.py* (Completed)

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class mySubscriberNode(Node):
    def __init__(self) -> None:
        super().__init__("my_subscriber")
        self.sub = self.create_subscription(String, 'my_topic', self.listener_callback, 10)
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

We are done with the content of the subscriber. Make sure that `my_topic` is the same as your publisher!

#### Updating the package with the new subscriber
Again, the ROS package `my_package` has no idea that it has a new executable! `ros2 run` does not work yet. We need to let them know. Add an entry point in `setup.py`.

```python
entry_points={
        'console_scripts': [
            'my_publisher = my_package.my_publisher:main',
            'my_subscriber = my_package.my_subscriber:main' # This line is new - you can just copy-paste this
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
1. Run subscriber: `ros2 run my_package my_subscriber`


![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/ros_pkg_overview.png)


## Understanting topics/messages

A **message is data** and a **topic is the channel** where nodes are subscribed to read messages or where the nodes publish those messages.

So far, we only focused on the *String* type of message and wrote a simple text. There are other message types which are used commonly in robotics projects such as *Pose*, *Position*, *Vector3*, *Twist*, etc. Each message type in ROS is defined in the respective library. For instance, the *String* is in the standard messages library as you see in the beginning of the previous codes: `from std_msgs.msg import String`.

The message types can be quite generic like in [geometry_msgs](https://docs.ros2.org/latest/api/geometry_msgs/index-msg.html) and [std_msgs](https://docs.ros2.org/latest/api/std_msgs/index-msg.html), or intended to be used in specific cases like in [sensor_msgs](https://docs.ros2.org/latest/api/sensor_msgs/index-msg.html) and [nav_msgs](https://docs.ros2.org/latest/api/nav_msgs/index-msg.html). You can also create your own message type, which will be discussed later.


### Turtlesim tutorial

In this part of the tutorial, we will learn about a very common topic `/cmd_vel` which often controls the velocity of a robot.

As we mentioned before, ROS has a sweet obsession with turtles. The logos of each ROS distribution has a turtle, the mobile robots which you will work on lab assignments are called *Turtlebot*s and the tutorial that we will do now is on *turtlesim*.

**Run the turtlesim node:** `ros2 run turtlesim turtlesim_node`. You will see a simulated turtle.

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/turtlesim-node.png)

{: .notice--info}
The `turtlesim` is a package that comes with ROS generic installation. You do not see a package named *turtlesim* under your `~/ros2_ws/src` directory but the code above works just fine! 
If you are curious, all the default packages are in */opt/ros/foxy/share*. You can use this command: `ros2 pkg prefix turtlesim`

In the ROS world, we can say that this turtle represents a mobile robot. We can control it as if it was a robot then. Luckily, the `turtlesim` package has an implemented *publisher node* that publishes `\cmd_vel` topic to the `turtlesim_node`.

**Run the teleoperation node:** `ros2 run turtlesim turtle_teleop_key`. You will be able to control the turtle with the arrow keys on your keyboard.

{: .notice--danger}
Make sure that the terminal which the `turtle_teleop_key` node is running is selected, NOT THE SIMULATION WINDOW. Otherwise, you cannot control the turtle.

### Visualize nodes and topics with rqt

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
    ```
    Type: geometry_msgs/msg/Twist
    Publisher count: 1
    Subscription count: 1
    ```

And there is much more that you can observe with `ros2 topic/param/service/node list/info` but these are enough for this tutorial. You will learn different [communication patterns of ROS](https://frdedynamics.github.io/hvl_robotics_website/courses/dat160/ros-comm-pattern) later. These will make more sense there.

One last cool thing is that you can see all these visually instead.

**Type:** `ros2 run rqt_graph rqt_graph`

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/rqt-graph.png)

{: .notice--info}
You can see what other executable nodes are available for **turtlesim** package by using the following command: `ros2 pkg executables turtlesim`

### Turtlesim and /cmd_vel exercise

This part is voluntary. 

Can you write a publisher that makes the turtle draw a circle?

<button id="toggleButton">Hint 1</button>
<div id="hiddenText" style="display: none; font-size: 0.5rem; line-height: 1.5;">
    Run both  <code>ros2 run turtlesim turtlesim_node</code> and <code>ros2 run turtlesim turtle_teleop_key</code>, and then check with <code>ros2 run rqt_graph rqt_graph</code> to see which topic might be relevant to publish?
</div>

<button id="toggleButton">Hint 2</button>
<div id="hiddenText" style="display: none; font-size: 0.5rem; line-height: 1.5;">
    Yes! Correct. It is **/turtle1/cmd_vel**. It means that you need to figure out the type (i.e message type) of this topic. You can now in a new terminal run <code>ros2 topic info /turtle1/cmd_vel</code>. When you find the message type, just Google it to see what fields this message type has.
</div>

<button id="toggleButton">Hint 3</button>
<div id="hiddenText" style="display: none; font-size: 0.5rem; line-height: 1.5;">
    You see that /turtle1/cmd_vel has two **Vector3** fields for *linear* and *angular*. If you click on Vector3 type, you see that it has *x*, *y* and *z* float fields. These are basic types we can just fill in in our Python script.
</div>

<button id="toggleButton">Hint 4: What values to write for a circle</button>
<div id="hiddenText" style="display: none; font-size: 0.5rem; line-height: 1.5;">
    The *linear* and *angular* velocity follows the frame system you learned in Peter Corke Chapter 2&3. If you think about the turtle as if it is a car or a *mobile robot*, then forward would be x-axis, right side would be y-axis and towards the screen would be the z-axis. You want your turtle move forward and rotate about the imaginary z-axis to draw a circle in 2D. It means your <code>msg.linear.x = 1.0</code> and <code>msg.angular.z = 1.0</code> another non-zero value depending on how fast you want your turtle to move and how big of a circle it should draw.
</div>


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

<button id="toggleButton">Doesn't it work?</button>
<div id="hiddenText" style="display: none; font-size: 0.5rem; line-height: 1.5;">
    1. Have you updated the setup.py?
    2. Have you colcon build and source (if you created a new file)?
    3. Does your autocomplete work?
</div>


### Turtlesim subscriber exercise

Can you add a subscriber to the `pose` topic to your cmd_vel publisher that prints out the x and y position as well as the current orientation of the turtle?

Extra Challenge: Can you use the pose subscriber to make the turtle draw an 8?

<button id="toggleButton">Click here to see the solution</button>
<div id="hiddenText" style="display: none;">
<pre><code class="python">
#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class DrawCircleNode(Node):
    def __init__(self):
        super().__init__("draw_circle")
        self.get_logger().info("DrawCircleNode created")
        self.current_pose = Pose()
        self.turn_dir = 1.0
        self.angles_traveled = 0.0

        self.create_subscription(Pose, '/turtle1/pose', self.clbk_pose, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer = self.create_timer(0.01, self.set_cmd_vel)

    def clbk_pose(self, msg):
        self.get_logger().info(f"position = [{msg.x}, {msg.y}]")
        self.get_logger().info(f"orientation = {msg.theta}")

        delta = msg.theta - self.current_pose.theta
        delta = (delta + math.pi) % (2 * math.pi) - math.pi
        self.angles_traveled += abs(delta)

        self.get_logger().info(f"angels traveled: {self.angles_traveled}")

        if self.angles_traveled >= (2*math.pi):
            self.angles_traveled  = 0.0
            self.turn_dir *= -1

        self.current_pose = msg


    def set_cmd_vel(self):
        msg = Twist()
        msg.angular.z = self.turn_dir
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