# Advanced UR {#Advanced-UR}

{: .notice--info}

This section is for information. Please conduct your lecturer/lab assistant before applying anything yourself.

## Theme

1. URCap
2. External Control
3. Communicating with UR5e using Python

## Equipment

1. UR5 / UR5e robot with PolyScope (at least version 5.1).
2. A pick and place setup (Lab-1 setup is suitable).
  

## Procedure

1. Make sure that the [URCap External Control](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_e_series.md) is installed on the robot.
2. Make sure that [RTDE Library](https://pypi.org/project/ur-rtde/) is installed on your PC. This must be installed in the virtual machine which is given to you. The documentation of the [ur_rtde](https://sdurobotics.gitlab.io/ur_rtde/installation/installation.html) package is attacheced to the link. The package is originally a C++ package but has full Python binding. Nonetheless, the package works best in Linux environment. 
3. Connect your PC and the robot to the same network. (Eduroam is problematic. Either use a local LAN or connect directly via ethernet cable.
4. Check your robot's IP from *Network settings*. 172.31.1.144 in Førde
5. Ping your robot to make sure that the connection is successful: `ping 172.31.1.144`
6. Check your PC's IP with `ifconfig` (for Ubuntu) or `ipconfig` (for Windows).
7. Make sure that the first 3 groups of the IP numbers matches (for example 172.31.1.XXX).
8. Make a program on UR:
  - Add **External Control** element (from the installation tab). Use your PC's IP number. 50002 is the external control port number.
  - Add a simple **Move** element.
9. Make a program on your PC:

  ```python
  import rtde_receive
  from rtde_control import RTDEControlInterface as RTDEControl

  from math import radians as d2r

  _ROBOT_IP = 172.31.1.144
  rtde_c = RTDEControl(_ROBOT_IP, RTDEControl.FLAG_USE_EXT_UR_CAP)
  rtde_r = rtde_receive.RTDEReceiveInterface(_ROBOT_IP)
  actual_q = rtde_r.getActualQ()  # read joint angles

  example_joints = [d2r(-175.82), d2r(-58.56), d2r(73.26), d2r(-11.08), d2r(19.57), d2r(-96.67)] ## Joint angles must be in radians
  rtde_c.moveJ(self.example_joints)

  ```

{: .notice--info}
If you want to use **MoveL**, make sure that you don't give the joint angles as input but (x,y,z, θ, ɸ, Ψ) TCP pose.

## Report

There is no need to hand in a report after this lab.

Signed attendance will suffice as approved lab exercise.

## Tasks

1. Pick-and-place example as you did in Lab-1. You may want to [check API](https://sdurobotics.gitlab.io/ur_rtde/api/api.html).
2. Move until contact example:
  ```python
    import rtde_control

    rtde_c = rtde_control.RTDEControlInterface("127.0.0.1")
    speed = [0, 0, -0.100, 0, 0, 0]
    rtde_c.moveUntilContact(speed)

    rtde_c.stopScript()
  ```


3.Force mode example: 

```python
  import rtde_control

  rtde_c = rtde_control.RTDEControlInterface("127.0.0.1")

  task_frame = [0, 0, 0, 0, 0, 0]
  selection_vector = [0, 0, 1, 0, 0, 0]
  wrench_down = [0, 0, -10, 0, 0, 0]
  wrench_up = [0, 0, 10, 0, 0, 0]
  force_type = 2
  limits = [2, 2, 1.5, 1, 1, 1]
  dt = 1.0/500  # 2ms
  joint_q = [-1.54, -1.83, -2.28, -0.59, 1.60, 0.023]

  # Move to initial joint position with a regular moveJ
  rtde_c.moveJ(joint_q)

  # Execute 500Hz control loop for 4 seconds, each cycle is 2ms
  for i in range(2000):
      t_start = rtde_c.initPeriod()
      # First move the robot down for 2 seconds, then up for 2 seconds
      if i > 1000:
          rtde_c.forceMode(task_frame, selection_vector, wrench_up, force_type, limits)
      else:
          rtde_c.forceMode(task_frame, selection_vector, wrench_down, force_type, limits)
      rtde_c.waitPeriod(t_start)

  rtde_c.forceModeStop()
  rtde_c.stopScript()

```
Intended movement:

![image-center](https://sdurobotics.gitlab.io/ur_rtde/_images/force_mode_example.gif)

