There are a few ways to create launch files in ROS2

[https://www.youtube.com/watch?v=dY9aZVMC-JM&ab_channel=HummingbirdRobotics](https://www.youtube.com/watch?v=dY9aZVMC-JM&ab_channel=HummingbirdRobotics)

or

[https://www.youtube.com/watch?v=xJ3WAs8GndA&ab_channel=RoboticsBack-End](https://www.youtube.com/watch?v=xJ3WAs8GndA&ab_channel=RoboticsBack-End)

## Preparing the [setup.py](http://setup.py) (?)

The `data_files[]` are supposed to be expanded. Just add the last two lines in the code piece below as well as the imports.

    import os
    from glob import glob
    
    data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            ((os.path.join('share', package_name, 'launch'), glob(os.path.join(package_name, 'launch', '*launch.[pxy][yma]*'))))
        ],

## Launch using Python

*/my_package/launch/my_launch.py*

    from launch import LaunchDescription
    from launch_ros.actions import Node
    
    def generate_launch_description():
        cmd_vel_pub = Node(
            package="my_package",
            executable="my_publisher", # We defined in the setup.py
            name="my_publisher_from_node" # new name of the node if you want
        )
    
        pose_log = Node(
            package="my_package",
            executable="my_subscriber",
            name="my_subscriber_from_node"
        )
    
        return LaunchDescription([cmd_vel_pub, pose_log])

or this might be cleaner:

    from launch import LaunchDescription
    from launch_ros.actions import Node
    
    def generate_launch_description():
    
        ld = LaunchDescription()
    
        cmd_vel_pub = Node(
            package="my_package",
            executable="my_publisher" # We defined in the setup.py
        )
    
        ld.add_action(cmd_vel_pub)
    
        pose_log = Node(
            package="my_package",
            executable="my_subscriber"
        )
    
        ld.add_action(pose_log)
    
        return ld

**********Note:**********

This might work also. It is cleaner

    setup(
        # Other parameters ...
        data_files=[
            # ... Other data files
            (os.path.join('share', package_name), glob('launch/*.launch.py'))
             # Include all config files.   
            (os.path.join('share', package_name), glob('config/*config.rviz'))  
        ]
    )