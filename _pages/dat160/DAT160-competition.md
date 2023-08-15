---
layout: single
title: "Competition"
permalink: /courses/dat160/competition
toc: true # The right side bar "on this page"
breadcrumbs: true  # the directory-of-documents type of header navigation
sidebar:
  nav: "dat160"  # the left navigation bar. Choose which category you want.
taxonomy: markup

my_variable: scripts.html
---

Semester projects on S&R robot competition

*This project represents 25% of your overall DAT160 grade. For more
info:*

[https://www.hvl.no/studier/studieprogram/emne/2023/dat160](https://www.hvl.no/studier/studieprogram/emne/2023/dat160)

# 1. Introduction

For the group project you will work as part of a team to solve a Search
and Rescue (S&R) scenario with a small team of mobile robots. Typical
skills to be acquired/demonstrated in the project include mobile robot
control, navigation and localisation, robot teams, robot software
architectures, and the Robot Operating System (ROS).

# 2. Group assignment

You will be asked to form groups of 2-3 students. **It is expected that
all group members contribute across all the parts of the group project,
as far as is feasible.** Note that the final report should include an
appendix with a log of what each group member worked on during the
project, in terms of theory, code, presentations, and report writing.
Inactive members of a group may fail the project.

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/dat160/competition/Picture1.jpg" alt="image" width="300" style="float: left;" />

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/dat160/competition/Picture2.png" alt="image" width="300" style="float: right;" />

*Figure 2. The robot team of 2x Turtlebots.*

# 4. Task to solve: S&R with a Robot Team

The task to solve for the project, and the final competition, is a
Search and Rescue (S&R) scenario with a team of 2 mobile robots
(Turtlebots), see Figure 1, with the following sensors:

- 1x forward-looking camera
  
- 1x lidar
  
- 1x IMU (gyro and accelerometer)
  
- 2x wheel encoders
  

The high-level goal is to provide assistance to injured persons in the
environment, as well as detect fire sources. The assistance needs to be
communicated by the robots, and is measured through a point system,
which is described in detail below.

The robots are able to communicate over a network using ROS topics,
services or actions. The robots have a limited amount of time to
complete the task, which is set to 10 minutes. There exists 5 variations
of the environment, each with different damage to the infrastructure.
For example, new obstacles, new openings in walls, targets at different
locations, and different mix of targets. See Figure 2. For the
competition one of the maps will be chosen at random.

The robots will be given a blueprint map of the environment but not the
locations of the Aruco-tags. Thus, the robot controllers need to be able
to handle a range of different situations, and pre-programming paths is
in general not possible.

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/dat160/competition/world1.png" alt="image" width="300" style="float: left;" />
<img src="{{ site.url }}{{ site.baseurl }}/assets/images/dat160/competition/world2.png" alt="image" width="300" style="float: left;" />
<img src="{{ site.url }}{{ site.baseurl }}/assets/images/dat160/competition/world3.png" alt="image" width="300" style="float: left;" />
<img src="{{ site.url }}{{ site.baseurl }}/assets/images/dat160/competition/world4.png" alt="image" width="300" style="float: left;" />
<img src="{{ site.url }}{{ site.baseurl }}/assets/images/dat160/competition/world5.png" alt="image" width="300" style="float: left;" />

*Figure 2. The 5 variations of the S&R environment.*

In your project report you need to clearly explain how the robots
communicate during S&R, which navigation algorithm is running and how
you implemented those elements. The navigation algorithm needs to be one
where you have done significant implementation yourself, and not an
existing one downloaded as is from the internet.

# 5. Report content and structure

The report should contain extensive and well-explained figures, but also
links to supplementary material supporting your project. The recommended
way to achieve this is to set up a free GitHub repository for the
project (<https://github.com>), where you can share code, printable
files, instructions and other material that helps document your project
fully.

**Good-quality videos, for example shared on YouTube, are required to
document the successful movement of the robot platforms, and should be
linked to from the report.**

Please note that you are yourself responsible for not publishing
copyrighted or illegal material on such channels, and you are advised to
only include content you yourself has produced, or material that is
under a suitable open source license, and where you have given suitable
attribution. For more information on the Creative Commons open source
licences: <https://creativecommons.org>

**Please note that the report will be submitted on WiseFlow, and that
there will be a plagiarism check.**

The report assignment should be submitted as a group, as **a single pdf
document**. It should be 3000 words plus/minus 10%. It should contain
the sections and suggested content (as a minimum) shown in appendix B.

# Appendix A -- Competition rules

## Introduction

You are supposed to develop a search and rescue system using mobile
robots in Gazebo environment. You are given a maze to solve containing 5
Aruco tags representing 3 features:

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/dat160/competition/human-ar.png" alt="image" width="300" style="float: left;" />
<img src="{{ site.url }}{{ site.baseurl }}/assets/images/dat160/competition/fire-ar.png" alt="image" width="300" style="float: left;" />
<img src="{{ site.url }}{{ site.baseurl }}/assets/images/dat160/competition/big-fire-ar.png" alt="image" width="300" style="float: left;" />
  

Your aim is to detect all the Aruco tags in the environment, inform
about their location and distinguish if it is a fire or a human. You
have 10 minutes (600 secs) in total to finish the task. Each
successfully detected Aruco tag will give you points.

- You can spawn as many robots as you want.
  
- Big fire requires at least 2 robots to meet at the fire location to
  extinguish. Therefore, robots should communicate with each other.
  

## Usage of software libraries and existing code

You are allowed to use existing libraries and external code, as long as
is documented in the report, and that you clearly explain 1) what the
library or piece of codes does, 2) how it is integrated with your own
code (include code in an appendix, and clearly label code that has been
copied from other sources), and 3) why what you actually have written in
terms of code is still is of suitable complexity for third year
information technology studies.

## Reporting targets

To receive points your robot(s) have to report their findings as per
below:

- Scoring starts when your system send a custom ROS request
  \`[rosparam set /start_flag True\`]{.mark}

```{=html}
<!-- -->
```

- [For each found]{.mark} [Aruco]{.mark} [tag,]{.mark}

> [you should set a parameter]{.mark} [called either \`\\ar_fire \` or
> \`\\ ar_human\`]{.mark} [as a dictionary which contains the]{.mark}
> [Aruco]{.mark} [id and robot's position (or]{.mark} [Aruco]{.mark}
> [tag's position).]{.mark}

- [The]{.mark} [reported]{.mark} [position has to be within a 2 meter
  radius of the actual position]{.mark} [to get the points. You only
  get one chance at reporting each ID with the correct
  position.]{.mark}

> Example in ROS node (Python):
> 
> [rospy.set_param(\'ar_fire\', \[{\'id\': 0, \'position\': {\'x\': 1,
> \'y\': 2, \'z\': 3}}\])]{.mark}
> 
> [rospy.set_param(\'ar_human\', \[{\'id\': 2, \'position\': {\'x\': 1,
> \'y\': 2, \'z\': 3}}\])]{.mark} LAURENZ

- Task ends in 3 conditions:
  
  - All 5 Aruco tags are found
    
  - Time is up
    
  - Robot [sets \`rosparam set /finish_flag True\`]{.mark}
    
- [Wrong]{.mark} [Aruco]{.mark} [tag ids will not give you a score
  (I.e.]{.mark} [if your]{.mark} [Aruco]{.mark} [tag id is not in
  the]{.mark} [Aruco]{.mark} [id list). However, if you identify
  the]{.mark} [Aruco]{.mark} [tag correctly yet the]{.mark} [position
  is wrong, then you will get a warning: A parameter /ar_fire_fail or
  /ar_human_fail identifying the]{.mark} [problematic ID.]{.mark}
  
- Note that there will be a running script to automatically keep the
  score. If the ROS parameters are not set properly, you will not get
  any points. The scoring script will be provided to you beforehand
  for testing purposes.
  

## Point system

Points are allocated according to the following rules:

- id: 0,1,3 are small fires (+100pt)
  
- id: 2 is human (+100pt)
  
- id:4 is big fire. For this tag the both robots should meet up
  (+100pt finding, +300pt if two robots meet there, i.e. if at any
  point after reporting id:4 the two robots are within a [3
  meter]{.mark} radius of this Aruco tag location)
  

## Final competition

- The final competition will run on a dedicated computer provided by
  the organizers. Each group should have a GitHub repo of their
  project, which is a private repo and unique name.
  
  - The last week, we will let them try their repo on the
    competition PC.
    
  - The competition PC will have the same VM that is provided to
    you.
    
- The controller should be launchable with a single launch file, which
  should be indicated to the organizers.
  
- The score will be maintained by a dedicated score-counting node
  controlled by the organizers.
  
- The environment is randomly chosen by the organizers from the given
  map pool.
  
- The winner of the competition will be the group that has got the
  best score in the chosen environment.
  
- Note that we will be also watching during the run so the timing of
  finding ARs, finishing the race etc. should also match with the eye
  test. So, it would be probably not necessary to find bugs (probably
  there are some) in the scoring script to get the highest point if
  you have such an intention even for fun :)
  

# Appendix B -- Suggested report structure

## Introduction
  
  - Brief introduction of robot teams for Search & Rescue, based on
    a literature search
    
  - Description of the task to perform, and requirements/rules in
    competition
    
  - A brief overview of your approach to the problem
    
## Design Process
  
  - What design iterations have you gone through?
    
  - Who worked on what, and when?
    
  - What problems did you encounter, and how did you solve them?
    
## Final implementation
  
  - Describe the final implementation of your robot software
    
  - Include figures with images/drawings/diagrams etc.
    
  - Be brief, include support material in appendices, if need be
    
## Experiments
  
  - What testing have you performed?
    
  - How well have you been able to perform the goal task during your
    testing? Show statistics if possible (i.e. average points on
    each environment, or similar), preferably in the form of a
    table.
    
  - What was the performance in the final competition? What went
    well, and what did not? Why?
    
## Conclusion
  
  - Briefly conclude on the work done
    
  - How would you make your solution better?
    

References

- Here you should put a numbered list of references to all sources you
  have used in the report, using IEEE referencing style
  
- These references should be cited from the text, using the IEEE
  citation format with citation number in square brackets, e.g. \[3\]
  for the third reference in the list
  
- For guidance on the IEEE referencing style, see for example:
  <https://www.bath.ac.uk/publications/library-guides-to-citing-referencing/attachments/ieee-style-guide.pdf>
  

## Appendices

- Obligatory log of what each group member worked on during the
  project, in terms of theory, code, presentations, and report
  writing. All members of the group have to sign this log.