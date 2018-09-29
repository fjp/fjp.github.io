---
layout: single
title:  "Trajectory Planning in the Frenet Space"
date:   2018-08-25 17:31:41 +0200
excerpt: "Planning mobile robot trajectories in structured environments using a reference path and Frenet coordinates."
categories: [robotics, trajectory planning, algorithms]
tags: [robotics, trajectory, algorithms, planning, frenet, coordinates, path]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Unscented Kalman Filter"
header:
  teaser: /assets/posts/2017-12-03-trajectory-following/minimize-error.png
  overlay_image: /assets/posts/2017-12-03-trajectory-following/minimize-error.png #keep it square 200x200 px is good
---


## Intoduction

There are many ways to plan a trajectory for a robot. A trajectory can be seen as a set of time ordered state vectors $x$.
The following algorithm introduces a way to plan trajectories to maneuver a mobile robot in a 2D plane.
It is specifically useful for structured environments, like highways, where a rough path, referred to as reference, is available a priori.

## Algorithm

1. **Determine the trajectory start state** \[x1,x2,theta,kappa,v,a\](0)
The trajectory start state is obtained by evaluating the previously calculated trajectory
at the prospective start state (low-level-stabilization).
At system initialization and after reinitialization, the current vehicle
position is used instead (high-level-stabilization).
2. **Selection of the lateral mode**
Depending on the velocity v the time based (d(t)) or running length / arc length based (d(s))
lateral planning mode is activated. By projecting the start state onto the reference curve the
the longitudinal start position s(0) is determined. The frenet state vector
\[s,ds,dds,d,d',d''\](0) can be determined using the frenet transformation.
For the time based lateral planning mode, \[dd, ddd\](0) need to be calculated.
3. **Generating the laterl and longitudinal trajectories**
Trajectories including their costs are generated for the lateral (mode dependent)
as well as the longitudinal motion (velocity keeping, vehicle following / distance keeping) in the frenet space.
In this stage, trajectories with high lateral accelerations with respect to the reference
path can be neglected to improve the computational performance.
4. **Combining lateral and longitudinal trajectories**
Summing the partial costs of lateral and longitduinal costs using
J(d(t),s(t)) = Jd(d(t)) + ks*Js(s(t)), for all active longidtuinal mode every
longitudinal trajectory is combined with every lateral trajectory and transfromed
back to world coordinates using the reference path. The trajectories are verified if they obey physical driving limits by
subsequent point wise evaluation of curvature and acceleration.
This leads to a set of potentially drivable maneuvers of a specific mode in world coordinates.
5. **Static and dynamic collision check**
Every trajectory set is evaluated with increasing total costs if static and dynamic
collisions are avoided. The trajectory with the lowest cost is then selected.
6. **Longitudinal mode alternation**
Using the sign based (in the beginning) jerk da(0), the trajectory with the
strongest decceleration or the trajectory which accelerates the least respectively
is selected and passed to the controller.


## Python code in Jupyter Notebook

{% include notebook path="/assets/notebooks/frenet.html" %}


### References

- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame](https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)
- [Moritz Werling Dissertation](https://www.ksp.kit.edu/download/1000021738)

#### Python Implementation:

- [Jupyter Notebook implementation](https://github.com/fjp/frenet/blob/master/ipython/python.ipynb)
- [https://github.com/AtsushiSakai/PythonRobotics#optimal-trajectory-in-a-frenet-frame](https://github.com/AtsushiSakai/PythonRobotics#optimal-trajectory-in-a-frenet-frame)
- [https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/FrenetOptimalTrajectory](https://github.com/AtsushiSakai/PythonRobotics#optimal-trajectory-in-a-frenet-frame)


### Matlab Implementation

- [https://github.com/fjp/frenet/tree/master/matlab](https://github.com/AtsushiSakai/PythonRobotics#optimal-trajectory-in-a-frenet-frame)
