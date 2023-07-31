# Advanced UR {#Advanced-UR}

::: warning::: titleWarning:::

Before connecting IO, turn the robot off and disconnect the power. Waita minute for any charge to dissipate. Or risk the unfortunate fate ofbeing human toast.:::

::: warning::: titleWarning:::

If we wear out the IO connections by abusing them, I will be a very sadpanda.:::

::: note::: titleNote:::

You can store your installation as you store a program. Changes to thedefault installation will be deleted.:::

## Theme

1. External E-stop
2. Digital IO for program flow
3. Relative waypoints
4. Built-in functions
5. Templates
6. Machine tending

## Equipment

1. UR5 / UR5e robot with PolyScope (they are different, old/new).
  
2. Robotiq 2-Finger Adaptive Robot Gripper / Hand-E Gripper
  
3. Lab station with
  
  > 1. E-stop, box with red button and 2 sets of wiring
  > 2. Workpiece holder
  > 3. Stack of workpieces
  > 4. Big box with sliding door.
  

## Before the lab

1. Complete the [Universal Robots Academy](./UR_exercises.html)**\*\<\-- this is very important!**\*
  
  > * \"3. Setting up a tool\"
  > * \"7. Safety settings\"
  > * \"9. Program Flow\"
  > * \"10. Feature Coordinates\"
  
2. Try to get together in a group of 2-4 people, plz.
  
3. Top tip for preparing: Use the [offline simulator fromUR](https://www.universal-robots.com/download/?option=41508&fbclid=IwAR1yU32_hPdsL40SljlNJBEC9J9uIRgfqNrnM8_6hQLOALupTzte9jB3-ss#section41493).For the ambitious; do all your programming in the simulator, bringyour saved program to the lab, bridge the simulation gap, success!
  

## Report

There is no need to hand in a report after this lab.

Signed attendance will suffice as approved lab exercise.

## Tasks

1. [Connect and test E-stop]()
2. [Connect Control box digital IO\'s, test IO\'s]()
3. [Elements of the Work Station]()
4. [Finite-State Sequence of the Machine Tending Box]()
5. [Make a program to open the machine door]()
6. [Make a program to close the machine door]()
7. [Make a program to pick up a workpiece]()
8. [Make a program to place a workpiece in the work station]()
9. [Make a program to place workpieces in a grid]()
10. [Combine everything to a complete machine tending program]()

## [Connect and test E-stop]{.title-ref}

The UR cabinet is ready for the E-stop to be connected using ascrewdriver. The E-stop IO\'s in the cabinet are all connected byshort-wires, giving the E-stop inputs high signals at all times. If, forany reason, this voltage goes away, an emergency stop is triggered justas when using the E-stop on the pendant.

The new E-stop contains 2 NC switches. If the red button is pressed, theswitches are opened, breaking the circuits.

1. Shutdown the robot
2. Disconnect power
3. Wait a minute
4. Connect the new E-stop
5. Fire up the robot again and verify that all E-stops work

## [Connect Control box digital IO\'s, test IO\'s]{.title-ref}

You will connect a total of 4 digital inputs to the UR, from the controlbox.

1. Shutdown the robot
2. Disconnect power
3. Wait a minute
4. Make the necessary connections to connect buttons and indicators todigital I/O.
5. Fire up the robot again
6. Test and name the inputs

## [Elements of the Work Station]{.title-ref}

{.align-center}

1. Door_open sensor
2. Door_close sensor
3. Workpiece_sensor
4. Clamp servo
5. Ready_LED and busy_LED

## [Finite-State Sequence of the Machine Tending Box]{.title-ref}

Set the machine to its initial state when starting up.

{.align-left}

## [Make a program to open the machine door]{.title-ref}

Make a program to open the machine door. This might sound easier than itis. Feel free to fasten the box. Consider using force control.

## [Make a program to close the machine door]{.title-ref}

Make a program to close the machine door. Same as above, but close.

## [Make a program to pick up a workpiece]{.title-ref}

Make a program to pick a workpiece. There are several ways to this.Consider the following suggestion as a starting point:

1. Move to a waypoint above the workpiece.
2. Close gripper.
3. Lower slowly relative to the waypoint above, until workpiece isdetected.
4. Relative move up 1 cm.
5. Open gripper.
6. Move to position.
7. Pick workpiece.

The direction option can be used to detect an object on contact andperform an action. Also, have a look at stack/de-stack options on theUR5e.

It is also possible to detect the force applied to the gripper oncontact, with pre-built functions. Have a look at them!

Tip: [my_variable = get_actual_tcp_pose()]{.title-ref} and add 0.01meters to the Z index [\[2\]]{.title-ref}, then MoveL to this variableposition.

## [Make a program to place a workpiece in the work station]{.title-ref}

Make a program to place a workpiece in the work station. Try to be asaccurate as possible when placing the workpiece!

## [Make a program to place workpieces in a grid]{.title-ref}

Make a program to place the workpieces in a grid. There are several waysto this. Consider the following suggestion as a starting point:

1. Make a waypoint at a corner of your grid.
2. Make a counter, row_num.
3. Increment counter each time a box is picked from machine.
4. Offset placement waypoints by using row_num.
5. Add col_num to the mix.

Also, take a look at palletize/de-palletize options on the UR5e for thebuilt-in template.

## [Combine everything to a complete machine tending program]{.title-ref}

Using the above programs as subprograms or copy/paste source, make aprogram to open the door, fetch the workpiece, place them in a grid,feed the machine a new workpiece and close the door. Wait until itsfinished. And repeat. One workpiece at a time.

{.align-center}

Voilà! Kick your feet up and let the robot do all the work!

## Questions

1. The E-stop has 2 NC switches. Why?
2. How many tasks did you complete? What kept you from completingeverything?
3. If the robot or the \"CNC\" changed location, how would you updatethe program?
4. What are the pros and cons between cobots and industrial robots?
5. Discuss how cobots can be applied to enhance workflow for a company,or in everyday life.
6. Does the configuration of the robot affect the applied force whenopening/closing the door?
7. Did you remember to have fun?