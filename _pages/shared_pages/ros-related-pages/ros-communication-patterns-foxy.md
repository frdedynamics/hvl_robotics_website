ROS provides different patterns that can be used to communicate between ROS Nodes:

* [Topics](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html): are used to send continuous data streams like e.g. sensor data. Data can be published on the topic independent of if there are any subscribers listening. Similarly, Nodes can also subscribe to topics independent of if a publisher exists. Topics allow for many to many connections, meaning that multiple nodes can publish and/or subscribe to the same topic. The official tutorial on how to implement a simple publisher/subscriber can be found [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html). How too create a custom message type can be found [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html).
* [Services](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html): are used when it is important that a message is received by the recipient and a response on the outcome is wanted. Services should only be used for short procedure calls e. g. changing the state of a system, inverse kinematics calculations or triggering a process. The official tutorial on how to implement a simple service server/client can be found [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html). How to create a custom action can be found [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html).
* [Actions](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html): are similar as services but are used if the triggered event needs more time and the possibility for receiving updates on the process or being able to cancel the process is wanted. An example could be when sending a navigation command. The official tutorial on how to implement an action server/client can be found [here](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html). An example which allows the client to cancel the goal can be found here: [server](https://github.com/ros2/examples/blob/foxy/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server.py), [client](https://github.com/ros2/examples/blob/foxy/rclpy/actions/minimal_action_client/examples_rclpy_minimal_action_client/client_cancel.py). How to create a custom action can be found [here](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Creating-an-Action.html)
* [Parameters](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html): are not actually a communication pattern but rather is a storage space for variables. It is not designed for high-performance and therefore mostly used for static variables like configuration parameters. The official guide on how to use parameters in a class can be found [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html).

## Exercise - Create a custom message package
1. Copy for the **communication_patterns** folder from the **ros2_students_25** repository into the src folder of your workspace.
2. Open a terminal and create a package called **communication_patterns_interfaces** by using the following commands:
   `cd ~/ros2_ws/src/communication_patterns`
   `ros2 pkg create --build-type ament_cmake communication_patterns_interfaces`
3. In the communication_patterns_interfaces package create the following folders: msg, srv and action

## Exercise - Create custom messages
1. Create a message type called *Position* that has three **float64** variables with the names: **x**, **y** and **z**
2. Create a message type called *Orientation* that has three **float64** variable with the names: **alpha**, **beta** and **gamma**
3. Create a message type called *Pose* that has two variable called **position** and **orientation** which have the previously created message types.
4. In the `CMakeLists.txt` file make sure to add the following:
    ```
    find_package(rosidl_default_generators REQUIRED)

    rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/MSG_TYPE_FILENAME"
    )
    ```
5. In the `package.xml` file make sure to add the following:   
    ```
    <buildtool_depend>rosidl_default_generators</buildtool_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
    ```
6. Don't forget to build your workspace!

## Exercise - Create a custom message publisher and subscriber
For this exercise you will be working in the file called `pose_publisher.py`
To test your code use: `ros2 launch communication_patterns pose_exercise.launch.py`.

<!-- ## Exercise - Create custom service messages
1. Add a service type with the name CheckCollision which has the following request fields: object_position and object_radius

## Exercise - Create a custom service client
For this exercise you will be working in the file called `robot_footprint_client.py`
To test your code use: `ros2 launch communication_patterns robot_footprint_exercise.launch.py`.

## Exercise - Create a custom service server
For this exercise you will be working in the file called `collision_server.py`
To test your code use: `ros2 launch communication_patterns collision_exercise.launch.py`. -->



