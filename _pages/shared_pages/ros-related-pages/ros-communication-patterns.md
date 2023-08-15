ROS provides different patterns that can be used to communcate between ROS Nodes:

Topics: are used to send continuous data streams like e.g. sensor data. Data can be published on the topic independent of if there are any subscribers listening. Similarly, Nodes can also subscribe to topics independent of if a publisher exists. Topics allow for many to many connections, meaning that multiple nodes can publish and/or subscribe to the same topic.

Services: are used when it is important that a message is recieved by the recipient and a response on the outcome is wanted. Services should only be used for short procedure calls e. g. changing the state of a system, inverse kinematics calculations or triggering a process.

Actions: are similar as services but are used if the triggered event needs more time and the possebility for recieving updates on the process or being able to cancel the process is wanted. An example could be when sending a navigation command.

Parameters: are not actually a communication pattern but rather is a storage space for variables. It is not designed for high-performance and therefore mostly used for static variables like configuration parameters.