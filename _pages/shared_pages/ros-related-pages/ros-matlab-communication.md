In this section, you will learn how to connect two powerful environments: MATLAB and ROS. The knowledge that you will gain in this section will be highly used in the [Open Manipulator Lab](https://frdedynamics.github.io/hvl_robotics_website/courses/ele306/tb2) and in your semester project.

By the end of this tutorial, you will be able to send commands from **Matlab installed your HOST PC** to your robot in **ROS in your virtual machine**, as well as receiving them.

# Prepare Matlab

You don't need to install Matlab on your VM. You will be using the Matlab on your host PC, which you have been using throughout the semester.

The ROS-Matlab communication is way easier than many of you might think. What you need is just [MATLAB ROS Toolbox](https://www.mathworks.com/products/ros.html). Please make sure that you have the toolbox installed: Home > Add-Ons > Manage Add-Ons:

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/matlab-toolbox.png)

# Prepare Virtual Machine
1. Select your virtual machine on the left bar
2. Click Edit virtual machine settings
3. Select Network Adapter
4. Select the first option **Bridged: Connected directly to the physical network** also check the **Replicate physical network connection state**
5. Go to Configure Adapters and ONLY select the wireless adapter which your PC has. In our case it is “Killer(R) Wi-Fi 6 AX1650 160MHz Wireless Network Adapter”
6. Save everything and start your virtual machine.

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/vm/VM-settings.png)

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/vm/vm_bridge_settings.png)

Next:
1. Start your virtual machine and open a new terminal: ``Ctrl + Alt + T`
2. Open the **.bashrc** using your favorite text editor: `gedit .bashrc`
3. Find the line where **ROS_DOMAIN_ID** is set: (for me: line 121: ``export ROS_DOMAIN_ID=24``)
4. Note the number somewhere. You will use this number in MATLAB.

# Publish a topic from MATLAB
At this point, you are quite free to choose what you want to control. It can be `turtlesim`, your custom robot or Open Manipulator joints. For simplicity, we will only control the `turtlesim` here but the concept is the same for all.

1. Start your node that you want to communicate: `ros2 run turtlesim turtlesim node`
2. Note the ROS topic that you want to publish/subscribe: `ros2 topic list`
3. Note the type of the message of this topic: `ros2 topic info /turtle1/cmd_vel`

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/turtlesim-topic-list.png)

Now, go back to MATLAB and create a script. Paste the code below:

*MATLAB_WORKSPACE/matlab_ros_publisher.m*
```matlab
test_publisher = ros2node("/test_vm_ros", 24);

cmdPub = ros2publisher(test_publisher, "turtle1/cmd_vel", "geometry_msgs/Twist");
cmdMsg = ros2message(cmdPub);
cmdMsg.linear.x = -0.2;
cmdMsg.linear.y = 0.0;
cmdMsg.linear.z = 0.0;
cmdMsg.angular.x = 0.0;
cmdMsg.angular.y = 0.0;
cmdMsg.angular.z = 0.0;


for cnt = 1:10
    send(cmdPub,cmdMsg)
    pause(1)
end
```

This script creates a node in domain number **24** and defines it as a publisher that publishes to the topic `turtle1/cmd_vel` for 10 seconds. You should modify this code according to your ROS_DOMAIN_ID and the topic you want to interact.

{: .notice--info}
If you want to use common ROS terminal commands such as `ros2 topic list` then you can use `setenv("ROS_DOMAIN_ID","24")` in your MATLAB terminal.

# Subscribe a topic by MATLAB

For this part, we will use the simple publisher that we created in the [ROS Intro](https://frdedynamics.github.io/hvl_robotics_website/courses/ele306/pub-sub#completing-the-publisher). 

1. Start your publisher: `ros2 run my_package my_publisher`
2. Decide which topic you want to subscribe to: `ros2 topic list`, `ros2 topic info `


The subscriber MATLAB script would look like:

*MATLAB_WORKSPACE/matlab_ros_subscriber.m*
```matlab
test_subscriber = ros2node("/test_vm_ros", 24);
msgSub = ros2subscriber(test_subscriber, "/topic", "std_msgs/String");

for cnt = 1:10
    receivedData = receive(msgSub, 10)
end
```

 

# Limitations
Imagine that you want to obtain the pose of the turtlesim on MATLAB. Then you need to subscribe to `turtle1/pose` topic which has **turtlesim/msg/Pose** message type. This message type is not available in ROS Toolbox. There are two things that you can do in this case:

## Option-1: Create a middleware node
You can create a middleware node in the ROS environment that subscribes to the *exotic* topic and publishes the information in it using more *generic* message type. 

In `turtle1/pose` example, th middleware node might look like this:

*YOUR_PREFERED_PACKAGE/turtle_pose_converter.py*
```python

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

## Pose msg type is very common in ROS but it uses quaternions for orientation
# If you want to skip the euler to quaternion conversion
# you can use Pose2D for simplicity
from geometry_msgs.msg import Pose 

from turtlesim.msg import Pose as Turtlesim_pose

import transforms3d


class myConverterNode(Node):
    def __init__(self) -> None:
        super().__init__("turtlesim_pose_converter")
        self.sub = self.create_subscription(Turtlesim_pose, '/turtle1/pose', self.listener_callback, 10)
        self.pub = self.create_publisher(Pose, '/turtle1/pose_converted', 10)
        self.create_timer(1.0, self.timer_callback)
        self.listened_pose = Turtlesim_pose()
        self.published_pose = Pose()

    def listener_callback(self, msg):
        # self.get_logger().info('turtlesim pose received')
        self.listened_pose = msg


    def timer_callback(self):
        self.published_pose.position.x = self.listened_pose.x
        self.published_pose.position.y = self.listened_pose.y
        self.published_pose.position.z = 0.0

        q = transforms3d.euler.euler2quat(0, 0, self.listened_pose.theta, 'rxyz')
        print(q[0],q[1], q[2], q[3]) # The order: q.w - q.x - q.y - q.z

        self.published_pose.orientation.w = q[0]
        self.published_pose.orientation.x = q[1]
        self.published_pose.orientation.y = q[2]
        self.published_pose.orientation.z = q[3]

        self.pub.publish(self.published_pose)
        self.get_logger().info('converted turtlesim pose published')

def main(args=None):
    rclpy.init(args=args)
    node = myConverterNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

And then, the MATLAB subscriber for this would be like:

*MATLAB_WORKSPACE/matlab_turtlesim_pose_subscriber.m*
```matlab
test_subscriber = ros2node("/test_vm_ros", 24);
msgSub = ros2subscriber(test_subscriber, "/turtle1/pose_converted", "geometry_msgs/Pose");

for cnt = 1:10
    poseData = receive(msgSub, 10)
    poseData.position
    poseData.orientation
end
```


## Option-2: Create custom message from ROS package
This is a better and systematic option, however, it is more cumbersome. You can learn the procedure following the [documentation](https://www.mathworks.com/help/ros/ug/create-custom-messages-from-ros-package.html).


