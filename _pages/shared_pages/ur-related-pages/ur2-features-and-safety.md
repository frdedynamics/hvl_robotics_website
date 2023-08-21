## Theme

1. Define features and safety plane
2. Define TCP
3. `for` and `if` loop
4. Play with the ping-pong ball dispensers! 🚀

## Equipment

1. UR5e robot with PolyScope.
2. Robotiq 2-Finger Adaptive Robot Gripper / Hand-E Gripper
3. Lab station with two ping-pong ball dispensers and 4 balls 🏓

## Before the lab

1.**Very important:** Complete the [Universal Robots Academy](https://academy.universal-robots.com/free-e-learning/e-series-e-learning/) 
{: .notice--warning}
  > 3. Setting up a tool
  > 7. Safety settings
  > 9. Program Flow
  > 10. Feature Coordinates
  
2.Try to get together in a group of 2-4 people.
{: .notice}
  

## Report

There is no need to hand in a report after this lab.
Signed attendance will suffice as approved lab exercise.

## Tasks

1. Set up a tool.
2. Define a safety plane.
3. Define a safety ball around the tool.

## Infinite ball pick and place

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ur/lab2_pic.png){: .align-center}

Initial conditions: Left (1) ball dispenser has 4 balls, right (2) balldispenser has 0 balls.

In this task you will program the robot to move one ping-pong ball at a time

{% capture notice-2 %}

**The program flow is the following**

1.Pick ball from 1. dispenser
  
  > * if ball not detected, halt
  
2.Place it in 2. dispenser
  
3.Repeat step 1-2 4 times
  
4.Pick balls from 2. dispenser
  
  > * if ball not detected, halt
  
5.Place balls in 1. dispenser
  
6.Repeat step 4-5 4 times
  
7.Loop for ever
{% endcapture %}

<div class="notice">{{ notice-2 | markdownify }}</div>  

The dispensers will be moved around when you\'re done. Therefore, make features for each dispenser and define waypoint like \"approach dispenser 1\" relative to dispenser 1\'s feature with an origin that makes sense. For more accuracy, consider using a reference when defining a feature.

And finally test your infinite loop!  🌈

## Questions

1. How will moveL motions differ if relative to your TCP, and ifrelative to flange?
2. What will happen if tool weight is entered too high or too low, andwhen?
3. Which direction does your TCP Z-axis point?
4. What did you use the safety plane for?
5. Did you make features for the dispensers?
6. Did stuff work after moving the dispensers around? Why?
