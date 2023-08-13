Welcome to the ROS section of the course. The purpose of these pages isto give you a compact ROS introduction where you can use in your courseproject. The documentation in this chapter is a target-based collectionof mainly the following very nice ROS tutorials.

::: seealsoFor a deeper understanding, they are highly recommended to be checkedout (OPTIONAL).



| Tables   | Are           | Cool  |
| -------- |:-------------:| -----:|
| col 1 is | left-aligned  | $1600 |
| col 2 is | centered      | $12   |
| col 3 is | right-aligned | $1    |



[Official ROS Documentation](https://docs.ros.org/en/foxy/)

* [TheConstructSim](https://www.theconstructsim.com/robotigniteacademy_learnros/ros-courses-library/)
* [A Concise Introduction to Robot Programming with ROS2](https://github.com/fmrico/book_ros2)

This page serves as a look-up page where you can find process andcommands easily.

## What is ROS?

* **ROS**: \"(Robot Operating System) is an open-source,meta-operating system for your robot. It provides the services you would expect from an operating system, including hardwareabstraction, low-level device control, implementation ofcommonly-used functionality, message-passing between processes, andpackage management. It also provides tools and libraries forobtaining, building, writing, and running code across multiplecomputers.\" [Follow this link for ros.org](https://www.ros.org)
  * Meta operating system
  * Fundamental components

<figure><img src="../_static/images/rosComponents.png" class="align-center"alt="../_static/images/rosComponents.png" /><figcaption>Source: <ahref="https://www.pishrobot.com/wp-content/uploads/2018/02/ROS-robot-programming-book-by-turtlebo3-developers-EN.pdf">ROSRobot Programming</a></figcaption></figure>

* ROS versions
  
  
  
  <figure><img src="../_static/images/rosVersions.jpg" class="align-center"alt="../_static/images/rosVersions.jpg" /><figcaption>Source: <ahref="https://twitter.com/_theconstruct_/status/1168418352802516992">ROSVersions</a></figcaption></figure>

### Important Shortcuts

* Open a new terminal: `Ctrl+Alt+T`
* Copy Paste: `Ctrl+Shit+C` and `Ctrl+Shit+V` (regular Ctrl+C andCtrl+V does not work in terminals)
* `Tab` for auto-complete
* Recursive search `Ctrl+R`

### Important Commands

* `roscore`: Start ROS master
* `rosrun my_pkg my_node`: Start a node (a .py or .cpp file)
* `roslaunch my_pkg my_launch.launch`: Start a launch file
* `catkin_make`: Compile catkin workspace
* `rosdep install --from-paths src --ignore-src -r -y`: Installdependencies of the packages in src
* `source ~/catkin_ws/devel/setup.bash`: loads the compiled shellenvironment (use after every `catkin_make`)
* `rospack profile`: checks and loads new packages (use after a newpackage is compiled)
* `rostopic echo/list/info (topic_name)`: Listen/list/get informationabout available topics
* `rosnode list/info (topic_name)`: List/get information aboutavailable nodes
* `rosmsg info msg_name`
* `roscd`: Change directory in ROS packages
* `rosparam list/load/get/set`: Commands for parameter server

### Dictionary

* **ROS master**: The main node enables other nodes to communicate
* **node**: ROS executable
* **launch**: Multiple ROS executables as well as parameters andarguments
* **publisher**: An executable providing data to the ROS system
* **subscriber**: An executable retrieving data from the ROS system
* **package**: A folder contains several nodes/launchers/resourcedeveloped for a specific purpose.
* **topic**: The channel between (a) publisher and (a) subscriber(more like a sophisticated data type definition)
* **message**: The information in a topic
* **gazebo**: A powerful simulator with physical properties (gravity,collision, lights/shadows etc)
* **rviz**: A visualization software without physical properties
* **rqt**: Many useful ROS-Qt packages like rqt_graph, rqt_publisher,rqt_controller_manager, rqt_plot etc.
* **urdf/sdf**: File formats to define a robot
* **parameter server**: A shared dictionary of parameters that nodesstore and retrieve at runtime
* **tf**: Stands for *Transform* and it is a powerful ROS packagebuilds relationships of multiple frames from a given robot model.

## Appendix

These are extra notes.

### ROS Cheat Sheet

[ROS CheatSheet](https://w3.cs.jmu.edu/spragunr/CS354_S19/handouts/ROSCheatsheet.pdf)

### ROS Terminology

This section explains the most frequently used ROS terms. Use thissection as a ROS glossary.

**ROS**

ROS provides standard operating system services such as hardwareabstraction, device drivers, implementation of commonly used featuresincluding sensing, recognizing, mapping, motion planning, messagepassing between processes, package management, visualizers and librariesfor development as well as debugging tools.

**Master**

The master acts as a name server for node-to-node connections andmessage communication. The command roscore is used to run the master,and if you run the master, you can register the name of each node andget information when needed. The connection between nodes and messagecommunication such as topics and services are impossible without themaster.

#### Node

A node refers to the smallest unit of processor running in ROS. Think ofit as one executable program. ROS recommends creating one single nodefor each purpose, and it is recommended to develop for easy reusability.For example, in case of mobile robots, the program to operate the robotis broken down into specialized functions. Specialized node is used foreach function such as sensor drive, sensor data conversion, obstaclerecognition, motor drive, encoder input, and navigation.

#### Package

A package is the basic unit of ROS. The ROS application is developed ona package basis, and the package contains either a configuration file tolaunch other packages or nodes. The package also contains all the filesnecessary for running the package, including ROS dependency librariesfor running various processes, datasets, and configuration file. Thenumber of official packages is about 2,500 for ROS Indigo as of July2017and about 1,600 packages for ROS Kinetic. In addition, althoughthere could be some redundancies, there are about 4,600 packagesdeveloped and released by users.

#### Metapackage

A metapackage is a set of packages that have a common purpose. Forexample, the Navigation metapackage consists of 10 packages includingAMCL, DWA, EKF, and map_server.

#### Message

A node sends or receives data between nodes via a message. Messages arevariables such as integer, floating point, and boolean. Nested messagestructure that contains another messages or an array of messages can beused in the message.

#### Topic

The topic is literally like a topic in a conversation. The publishernode first registers its topic with the master and then startspublishing messages on a topic. Subscriber nodes that want to receivethe topic request information of the publisher node corresponding to thename of the topic registered in the master. Based on this information,the subscriber node directly connects to the publisher node to exchangemessages as a topic.

#### Publish and Publisher

The term 'publish' stands for the action of transmitting relativemessages corresponding to the topic. The publisher node registers itsown information and topic with the master, and sends a message toconnected subscriber nodes that are interested in the same topic. Thepublisher is declared in the node and can be declared multiple times inone node.

#### Subscribe and Subscriber

The term 'subscribe' stands for the action of receiving relativemessages corresponding to the topic. The subscriber node registers itsown information and topic with the master, and receives publisherinformation that publishes relative topic from the master. Based onreceived publisher information, the subscriber node directly requestsconnection to the publisher node and receives messages from theconnected publisher node. A subscriber is declared in the node and canbe declared multiple times in one node.

The topic communication is an asynchronous communication which is basedon publisher and subscriber, and it is useful to transfer certain data.Since the topic continuously transmits and receives stream of messagesonce connected, it is often used for sensors that must periodicallytransmit data. On the other hands, there is a need for synchronouscommunication with which request and response are used. Therefore, ROSprovides a message synchronization method called 'service'. A serviceconsists of the service server that responds to requests and the serviceclient that requests to respond. Unlike the topic, the service is aone-time message communication. When the request and response of theservice is completed, the connection between two nodes is disconnected.

#### Service

The service10 is synchronous bidirectional communication between theservice client that requests a service regarding a particular task andthe service server that is responsible for responding to requests.

#### Service Server

The *service server* is a server in the service message communicationthat receives a request as an input and transmits a response as anoutput. Both request and response are in the form of messages. Upon theservice request, the server performs the designated service and deliversthe result to the service client as a response. The service server isimplemented in the node that receives and executes a given request.

#### Service Client

The *service client* is a client in the service message communicationthat requests service to the server and receives a response as an input.Both request and response are in the form of message. The client sends arequest to the service server and receives the response. The serviceclient is implemented in the node which requests specified command andreceives results.

#### Action

The action11 is another message communication method used for anasynchronous bidirectional communication. Action is used where it takeslonger time to respond after receiving a request and intermediateresponses are required until the result is returned. The structure ofaction file is also similar to that of service. However, feedback datasection for intermediate response is added along with goal and resultdata section which are represented as request and response in servicerespectively. There are action client that sets the goal of the actionand action server that performs the action specified by the goal andreturns feedback and result to the action client.

#### Action Server

The *action server* is in charge of receiving goal from the client andresponding with feedback and result. Once the server receives goal fromthe client, it performs predefined process.

#### Action Client

The *action client* is in charge of transmitting the goal to the serverand receives result or feedback data as inputs from the action server.The client delivers the goal to the action server, then receivescorresponding result or feeedback, and transmits follow up instructionsor cancel instruction.

#### Parameter

The parameter in ROS refers to parameters used in the node. Think of itas \*\* *.ini*\* configuration files in Windows program. Default valuesare set in the parameter and can be read or written if necessary. Inparticular, it is very useful when configured values can be modified inreal-time. For example, you can specify settings such as USB portnumber, camera calibration parameters, maximum and minimum values of themotor speed.

#### Parameter Server

When parameters are called in the package, they are registered with theparameter server which is loaded in the master.

#### Catkin

The catkin refers to the build system of ROS. The build system basicallyuses CMake (Cross Platform Make), and the build environment is describedin the 'CMakeLists.txt' file in the package folder. CMake was modifiedin ROS to create a ROS-specific build system. Catkin started the alphatest from ROS Fuerte and the core packages began to switch to Catkin inthe ROS Groovy version. Catkin has been applied to most packages in theROS Hydro version. The Catkin build system makes it easy to useROS-related builds, package management, and dependencies among packages.If you are going to use ROS at this point, you should use Catkin insteadof ROS build (rosbuild).

#### ROS Build

The ROS build is the build system that was used before the Catkin buildsystem. Although there are some users who still use it, this is reservedfor compatibility of ROS, therefore, it is officially not recommended touse. If an old package that only supports the rosbuild must be used, werecommend using it after converting rosbuild to catkin.

#### roscore

Roscore is the command that runs the ROS master. If multiple computersare within the same network, it can be run from another computer in thenetwork. However, except for special case that supports multipleroscore, only one roscore should be running in the network. When ROSmaster is running, the URI address and port number assigned forROS_MASTER_URI environment variables are used. If the user has not setthe environment variable, the current local IP address is used as theURI address and port number 11311 is used which is a default port numberfor the master.

#### rosrun

Rosrun is the basic execution command of ROS. It is used to run a singlenode in the package. The node uses the ROS_HOSTNAME environment variablestored in the computer on which the node is running as the URI address,and the port is set to an arbitrary unique value.

#### roslaunch

While rosrun is a command to execute a single node, roslaunch18 incontrast executes multiple nodes. It is a ROS command specialized innode execution with additional functions such as changing packageparameters or node names, configuring namespace of nodes, settingROS_ROOT and ROS_PACKAGE_PATH, and changing environment variables19 whenexecuting nodes. roslaunch uses the \*\* *.launch*\* file to definewhich nodes to be executed. The file is based on XML (Extensible MarkupLanguage) and offers a variety of options in the form of XML tags.

#### bag

The data from the ROS messages can be recorded. The file format used iscalled bag20, and \*\* *.bag*\* is used as the file extension. In ROS,bag can be used to record messages and play them back when necessary toreproduce the environment when messages are recorded. For example, whenperforming a robot experiment using a sensor, sensor values are storedin the message form using the bag. This recorded message can berepeatedly loaded without performing the same test by playing the savedbag file. Record and play functions of rosbag are especially useful whendeveloping an algorithm with frequent program modifications.

#### ROS Wiki

ROS Wiki is a basic description of ROS based on[Wiki](http://wiki.ros.org/) that explains each package and the featuresprovided by ROS. This Wiki page describes the basic usage of ROS, abrief description of each package, parameters used, author, license,homepage, repository, and tutorial. The ROS Wiki currently has more than18,800 pages of content.

#### Repository

An open package specifies repository in the Wiki page. The repository isa URL address on the web where the package is saved. The repositorymanages issues, development, downloads, and other features using versioncontrol systems such as svn, hg, and git. Many of currently availableROS packages are using GitHub21 as repositories for source code. Inorder to view the contents of the source code for each package, checkthe corresponding repository.

#### Graph

The relationship between nodes, topics, publishers, and subscribersintroduced above can be visualized as a graph. The graphicalrepresentation of message communication does not include the service asit only happens one time. The graph can be displayed by running the'rqt_graph' node in the 'rqt_graph' package. There are two executioncommands, 'rqt_graph' and 'rosrun rqt_graph rqt_graph'.

#### Name

Nodes, parameters, topics, and services all have names. These names areregistered on the master and searched by the name to transfer messageswhen using the parameters, topics, and services of each node. Names areflexible because they can be changed when being executed, and differentnames can be assigned when executing identical nodes, parameters,topics, and services multiple times. Use of names makes ROS suitable forlarge-scale projects and complex systems.

#### Client Library

ROS provides development environments for various languages by usingclient library23 in order to reduce the dependency on the language used.The main client libraries are C++, Python, Lisp, and other languagessuch as Java, Lua, .NET, EusLisp, and R are also supported. For thispurpose, client libraries such as roscpp, rospy, roslisp, rosjava,roslua, roscs, roseus, PhaROS, and rosR have been developed.

#### URI

A URI (Uniform Resource Identifier) is a unique address that representsa resource on the Internet. The URI is one of basic components thatenables interaction with Internet and is used as an identifier in theInternet protocol.

#### CMakeLists.txt

Catkin, which is the build system of ROS, uses CMake by default. Thebuild environment is specified in the 'CMakeLists.txt' file in eachpackage folder.

#### package.xml

An XML file contains package information that describes the packagename, author, license, and dependent packages.

::: seealsoNot everything is included here. For more information and detailedexplanation, please see [ROS RobotProgramming](https://www.pishrobot.com/wp-content/uploads/2018/02/ROS-robot-programming-book-by-turtlebo3-developers-EN.pdf)Chapter 4.1.:::

### Message Communication in ROS

Here is the ROS message communication.

<figure><img src="../_static/images/rosMsgCommunication.png"class="align-center" alt="../_static/images/rosMsgCommunication.png" /><figcaption>Source: ROS Robot Programming <em>(Book)</em></figcaption></figure>