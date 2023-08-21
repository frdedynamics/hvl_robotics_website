
{: .notice--danger}

**Danger:** Before connecting IO, turn the robot off and disconnect the power. Waita minute for any charge to dissipate. Or risk the unfortunate fate of being human toast! 🍞

{: .notice--warning}

**Warning:** If you wear out the IO connections by abusing them, shame on you! 🔔 shame! 🔔 shame!

{: .notice--info}

You can store your installation as you store a program. Changes to the default installation will be deleted.

## Theme

1. External E-stop
2. Digital IO for program flow
3. Relative waypoints
4. Built-in functions
5. Templates
6. Machine tending

## Equipment

1. UR5e robot with PolyScope.
  
2. Robotiq 2-Finger Adaptive Robot Gripper / Hand-E Gripper
  
3. Lab station with
  > 1. E-stop, box with red button and 2 sets of wiring
  > 2. Workpiece holder
  > 3. Stack of workpieces
  > 4. Big box with sliding door.
  

## Before the lab

1.**Very important:** Complete the [Universal Robots Academy](https://academy.universal-robots.com/free-e-learning/e-series-e-learning/) 
{: .notice--warning}
  > 3. Setting up a tool
  > 7. Safety settings
  > 9. Program Flow
  > 10. Feature Coordinates
  
2.Try to get together in a group of 2-4 people.
{: .notice}

3.Top tip for preparing: Use the [offline simulator fromUR](https://www.universal-robots.com/download/?option=41508&fbclid=IwAR1yU32_hPdsL40SljlNJBEC9J9uIRgfqNrnM8_6hQLOALupTzte9jB3-ss#section41493). For the ambitious; do all your programming in the simulator, bring your saved program to the lab, bridge the simulation gap, success!  🌈  Or just wait for the lab to do it directly on the real robot...
{: .notice}  

## Report

There is no need to hand in a report after this lab.
Signed attendance will suffice as approved lab exercise.

## Tasks

### Connect and test the E-stop

The UR cabinet is ready for the E-stop to be connected using a screwdriver. The E-stop IO\'s in the cabinet are all connected by short-wires, giving the E-stop inputs high signals at all times. If, for any reason, this voltage goes away, an emergency stop is triggered just as when using the E-stop on the pendant.

The new E-stop contains 2 NC switches. If the red button is pressed, the switches are opened, breaking the circuits.

1. Shutdown the robot
2. Disconnect power
3. Wait a minute
4. Connect the new E-stop
5. Fire up the robot again and verify that all E-stops work

### Connect Control box digital IO\'s, test IO\'s

You will connect a total of 4 digital inputs to the UR, from the controlbox.

1. Shutdown the robot
2. Disconnect power
3. Wait a minute
4. Make the necessary connections to connect buttons and indicators todigital I/O.
5. Fire up the robot again
6. Test and name the inputs

### Elements of the Work Station

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ur/URlab3_1.png){.align-center}

1. Door_open sensor
2. Door_close sensor
3. Workpiece_sensor
4. Clamp servo
5. Ready_LED and busy_LED

### Finite-State Sequence of the Machine Tending Box

Set the machine to its initial state when starting up.

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ur/URlab3_2.png){.align-center}

### Make a program to open the machine door

Make a program to open the machine door. This might sound easier than it is. Feel free to fasten the box. Consider using force control.

### Make a program to close the machine door

Make a program to close the machine door. Same as above, but closed.

### Make a program to pick up a workpiece

Make a program to pick a workpiece. There are several ways to do this. Consider the following suggestion as a starting point:

1. Move to a waypoint above the workpiece.
2. Close gripper.
3. Lower slowly relative to the waypoint above, until workpiece is detected.
4. Relative move up 1 cm.
5. Open gripper.
6. Move to position.
7. Pick workpiece.

The direction option can be used to detect an object on contact and perform an action. Also, have a look at stack/de-stack options on the UR5e.

It is also possible to detect the force applied to the gripper on contact, with pre-built functions. Have a look at them!

Tip: *my_variable = get_actual_tcp_pose()* and add 0.01meters to the Z index, then MoveL to this variableposition.

### Make a program to place a workpiece in the work station

Make a program to place a workpiece in the work station. Try to be as accurate as possible when placing the workpiece!

### Make a program to place workpieces in a grid

Make a program to place the workpieces in a grid. There are several ways to do this. Consider the following suggestions as a starting point:

1. Make a waypoint at a corner of your grid.
2. Make a counter, row_num.
3. Increment counter each time a box is picked from machine.
4. Offset placement waypoints by using row_num.
5. Add col_num to the mix.

Also, take a look at palletize/de-palletize options on the UR5e for thebuilt-in template.

### Combine everything to a complete machine tending program

Using the above programs as subprograms or copy/paste source, make a program to open the door, fetch the workpiece, place them in a grid, feed the machine a new workpiece and close the door. Wait until it's finished. And repeat. One workpiece at a time.

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ur/URlab3_3.png){.align-center}

Voilà! Kick your feet up and let the robot do all the work!

## Questions

1. The E-stop has 2 NC switches. Why?
2. How many tasks did you complete? What kept you from completingeverything?
3. If the robot or the \"CNC\" changed location, how would you updatethe program?
4. What are the pros and cons between cobots and industrial robots?
5. Discuss how cobots can be applied to enhance workflow for a company,or in everyday life.
6. Does the configuration of the robot affect the applied force whenopening/closing the door?
7. Did you remember to have fun?
