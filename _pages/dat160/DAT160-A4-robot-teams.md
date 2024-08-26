---
layout: single
title: "Robot Teams"
permalink: /courses/dat160/a4_old
toc: true # The right side bar "on this page"
breadcrumbs: true  # the directory-of-documents type of header navigation
sidebar:
  nav: "dat160"  # the left navigation bar. Choose which category you want.
taxonomy: markup
---

This assignment should warm you up with your reflection on both robot and multi-robot software architectures. The assignment is related to your semester project, hence the answers depends on your reflection about your solution and design choices.

The submission is individual. Since you are working in teams in the software project, it is expected that your answers deal with the same solutions. However, your reflection about the solutions/design choices, and your justification for these reflections, are individual.

You will be allowed to consolidate your answers and incorporate them as an appendix before the deadline for submitting your project reports (8th December).

# (Muli)Robot architecture
You will use at least one robot in your semester project, meaning that we are dealing with a
multi-robot system (MRS). Make sure that you justify your answers, e.g. if you say that your
coordination is distributed and communication is implicit, you should also say why you think
it is the case.

## Architecture
Use the book chapter 12.3 and the article “An Architecture for Decentralized, Collaborative,
and Autonomous Robots” to analyze your architecture. For this question, it does not maOer
whether you have utilized a single-robot or multi-robot system. More precisely, use

1. Chapter 12.3. to characterize the layers (planning, behavior, executive) of your
architecture. Tell also whether you combine/integrate layers.
2. Chapter 12.3.3. to identify which kind of planning (HTN or Planning/scheduler) you
use in your solution.
3. Table 2 in the article to decide which challenges your solution is tackling, and which
type of architecture you are using (OO-R, CB-R, SD-R, etc.)

## Coordination
Use the article “Multi Robot Coordination Analysis, Taxonomy, Challenges and Future Scope”
and the book chapter 53.3. to analyze your MRS coordination and communication
mechanism. More precisely, use 

1. Table 1 and specify the parameters for our MRS  (e.g. composition: homogeneous -
heterogeneous, control architecture: decentralized - centralized - hybrid, etc.)
1. Figure 1 and specify which kind of MRS you use (e.g. coordinated, homogeneous,
competitive, etc.)
1. Figure 2 and specify which kind of coordination you use (distributed, decentralized,
weak, dynamic, etc)
1. The book chapter to tell what kind of communication you use (stigmetry, excplicit,
etc.)

# Task allocation
Use the article “Multi-robot Task Allocation: A Review of the State-of-the-Art” to decide on
the task allocation approach you are using in your solution. More precisely, use

1. The taxonomy in section 3 to decide whether you are dealing with an ST-SR-IA, MT-
MT-TA, etc.
2. The organizational paradigms in section 4 to categorize and tell whether your solution
use a centralized or a decentralized approach.
3. The optimization approach in section 5 to characterize your approach, e.g. is a
market/auction based or optimization based.

## Task and motion planning
Use the article “Multiple Mobile Task and Motion Planning: A Survey” to analyze the problem
representation. More precisely, use
