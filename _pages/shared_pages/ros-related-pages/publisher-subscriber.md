

## How does publisher/subscriber work?
- Make a schematic
- explain topics, services, parameters.

# Publisher/Subscriber

~/ros2_ws/src/my_package/my_package/my_node.py

## /src folder

Create with an empty `__init__.py`.

## Simple Node:

This is a simple Python node under /src

    def main():
        print('Hi from my_package.')
    
    if __name__ == '__main__':
        main()

This is a simple publisher which does nothing:

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

ATM, it works as a regular Python script but not as a ROS node. There is no autocomplete and `ros2 run` does not work.

We need to add entry point in `setup.py`:

    entry_points={
            'console_scripts': [
                'my_node = my_package.my_node:main'
            ],
        },
    )

and then compile: `colcon build` and source.

* File name of the node
* node name in the code
* and the executable name in the `[setup.py](http://setup.py)`

are not necessarily the same. They can be different but it is easier to follow if all are the same.

****************************************Note for those who know ROS:****************************************

Normally, you wouldn’t need to `catkin_make` after you make a change in a Python node. It is not the case in ROS2. You always need to `colcon build`.

But there is a tweak around. After you create your ROS2 node in the package, compiled it once (or not), you can run `colcon build --symlink-install`and source bashrc, then the changes in this node will be affected immediately.

### ************************Create timer************************

Create a timer function, call it in the init() and spin in the main()

*/my_package/src/my_node.py*

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

* It is nice to check out the **********************leaf topics********************** in `rqt_graph` to see unpublished/unsubscribed topics also.

### Finally the setup.py:

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