ROS provides different patterns that can be used to communcate between ROS Nodes:

* [Topics](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html): are used to send continuous data streams like e.g. sensor data. Data can be published on the topic independent of if there are any subscribers listening. Similarly, Nodes can also subscribe to topics independent of if a publisher exists. Topics allow for many to many connections, meaning that multiple nodes can publish and/or subscribe to the same topic. The official tutorial on how to implement a simple publisher/subscriber can be found [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html).
* [Services](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html): are used when it is important that a message is recieved by the recipient and a response on the outcome is wanted. Services should only be used for short procedure calls e. g. changing the state of a system, inverse kinematics calculations or triggering a process. The official tutorial on how to implement a simple service server/client can be found [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html).
* [Actions](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html): are similar as services but are used if the triggered event needs more time and the possebility for recieving updates on the process or being able to cancel the process is wanted. An example could be when sending a navigation command. The official tutorial on how to implement an action server/client can be found [here](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html).
* [Parameters](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html): are not actually a communication pattern but rather is a storage space for variables. It is not designed for high-performance and therefore mostly used for static variables like configuration parameters. The official guide on how to use parameters in a class can be found [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html).

# Topics

## Defining a message type

## Publisher

## Subscriber

# Services

## Defining a service

## Service Server

## Service Client


# Actions

## Defining an action

## Action Server

```python
import rclpy
from rclpy.node import Node
from robot_teleop_interfaces.action import ReplayVel
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

class ReplayVelocitiesActionServer(Node):
    def __init__(self):
        super().__init__('replay_velocities_action_server')

        # Definition of the action server
        self._action_server = ActionServer(
            self,
            ReplayVel,
            '/replay_velocities',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

def main(args=None):
    rclpy.init(args=args)

    replay_vel_action_server = ReplayVelocitiesActionServer()

    #In order to be able to cancel goals while they are being executed a MultiThreadedExecutor is needed for the action server. 
    executor = MultiThreadedExecutor()
    rclpy.spin(replay_vel_action_server, executor=executor)


if __name__ == '__main__':
    main()
```


### Callback Functions

The `goal_callback` function contains any logic that decides if a goal request is accepted or declined. The following example simply accepts all goal requests.
```python
from rclpy.action import GoalResponse

def goal_callback(self, goal_request):
    """Accept or reject a client request to begin an action."""
    self.get_logger().info('Received goal request')
    return GoalResponse.ACCEPT
```

The `cancel_callback` function contains the logic that decides to accept or reject a request to cancel the current goal. In the following example all cancel requests are accepted.
```python
from rclpy.action import CancelResponse

def cancel_callback(self, goal_handle):
    """Accept or reject a client request to cancel an action."""
    self.get_logger().info('Received cancel request')
    return CancelResponse.ACCEPT
```

The `execute_callback` function is the function is the main function of the action server and contains the code that works towards achieving the set goal. Below is an example of an `execute_callback`. For the example code the action `ReplayVel` is used. Check [here]() how to create an action.
```python
from robot_teleop_interfaces.action import ReplayVel

async def execute_callback(self, goal_handle):
    self.get_logger().info('Executing goal...')
    req = goal_handle.request
    robot_id = req.robot_id
    feedback_msg = ReplayVel.Feedback()

    for cmd_vel in req.cmd_vel_list:
        self.robot_vel_publisher.publish(cmd_vel)
        feedback_msg.current_pose = self.current_poses[robot_id]
        goal_handle.publish_feedback(feedback_msg)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            return ReplayVel.Result()

        time.sleep(0.1)

    goal_handle.succeed()

    result = ReplayVel.Result()
    result.final_pose = self.current_poses[robot_id]
    return result
```


## Action Client

**Define action client**
```python
self._action_client = ActionClient(self, ReplayVel, '/replay_velocities')
self._action_client.wait_for_server()
```

**Send goal request to action server**
```python
self.replay_vel_goal_msg = ReplayVel.Goal()
self.replay_vel_goal_msg.robot_id = self.robot_id
self.replay_vel_goal_msg.cmd_vel_list = copy.deepcopy(self.cmd_vel_lists[self.robot_id])

self._send_goal_future = self._action_client.send_goal_async(self.replay_vel_goal_msg, feedback_callback=self.feedback_callback)
self._send_goal_future.add_done_callback(self.goal_response_callback)
```

**Send request to cancel goal to action server**
```python
self._cancel_goal_future = self.goal_handle.cancel_goal_async()
self._cancel_goal_future.add_done_callback(self.cancel_done_callback)
```

**Callback function for response to goal request**
```python
def goal_response_callback(self, future):
    self.goal_handle = future.result()
    if not self.goal_handle.accepted:
        self.get_logger().info('Goal rejected')
        return

    self.get_logger().info('Goal accepted')

    self._get_result_future = self.goal_handle.get_result_async()
    self._get_result_future.add_done_callback(self.get_result_callback)
```

**Callback function for receiving the final result of the action server**
```python
def get_result_callback(self, future):
    self.get_logger().info('get_result_callback')
    result = future.result().result
    self.get_logger().info('Result: {0}'.format(result.final_pose.position))
    self.replay_demonstration = False
    self.wait_for_replay = False
```

**Callback function for receiving feedback from the action server while the goal is being executed**
```python
def feedback_callback(self, feedback_msg):
    feedback = feedback_msg.feedback
    self.get_logger().info('Received feedback: {0}'.format(feedback.current_pose.position))
```

**Callback function for the response from the action server to the cancel request**
```python
def cancel_done_callback(self, future):
    cancel_response = future.result()
    if len(cancel_response.goals_canceling) > 0:
        self.get_logger().info('Goal successfully canceled')
        self.replay_demonstration = False
        self.wait_for_replay = False
        self.cancel_replay = False
    else:
        self.get_logger().info('Goal failed to cancel')
```