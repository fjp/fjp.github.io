---
layout: single
title:  "Porbabilistic Robotics Overview"
date:   2021-08-18 15:00:42 +0200
excerpt: "Overview of algorithms in probabilistic robotics."
permalink: /posts/probabilistic-robotics/
categories: [robotics, mapping, localization, slam, ros, probability]
tags: [robotics, localization, slam, monte carlo localization, amcl, fastslam, ros, gird, mapping, map, probability, probabilistic]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Kalman filter"
header-includes:
  - \usepackage{algorithm2e}
header:
  teaser: https://images-na.ssl-images-amazon.com/images/I/51Q2dvR+eTL._SX424_BO1,204,203,200_.jpg
  overlay_image: https://images-na.ssl-images-amazon.com/images/I/51Q2dvR+eTL._SX424_BO1,204,203,200_.jpg #keep it square 200x200 px is good
---

The following sections summairze the Grid-based FastSLAM algorithm which is one instance of FastSLAM. This algorithm estimates the trajectory of a mobile robot while simultaneously creating a grid map of the environment.
Grid-based FastSLAM is combination of a particle filter such as Adaptive [Monte Carlo Localization](/posts/localization/mcl/) (amcl) and a mapping algorithm such as occupancy grid mapping. 

## Basics

### Introduction



### Recursive State Estimation

#### Robot Environment Interaction

##### State

Environments are characterized by state. State that change over time is called dynamic state, e.g., moving people or other vehicles.
Static state is non-changing state, such as the location of walls in (most) buildings. The state also includes variables regarding the robot itself, such as its pose, velocity, whether or not its sensors are functioning correctly and so on. State is denoted by $x$ and the state at time $t$ by $x_t$. Typical state variables include:

- Robot **pose**
- In robot manipulation, the pose includes variables for the **configuration of robot's actuators** e.g., joint angles. Degrees of freedom is related to the **kinematic state** of a robot.
-  Robot **velocity** and **velocities of its joints** are commonly referred to as **dynamic state**.
-  **Locations and features of surrounding objects** in the environment are also state variables e.g., trees, walls. In some problems, objects will assume the form of **landmarks**, which are distinct, staationary features of the environment that can be recognized reliably.
-  **Location and velocity of moving objects** and people are also potential state variables.
-  Broken sensors or level of battery can be state variables.
-  

##### Environment Interaction


##### Probabilistic Generative Laws

##### Belief Distributions

#### Bayes Filter

- Bayes Filter Algorithm
- Markov Assumption

### Gaussian Filters

### Nonparametric Filters

### Robot Motion

### Robot Perception

## Localization

### Mobile Robot Localization: Markov and Gaussian

### Mobile Robot Localization: Grid And Monte Carlo

## Mapping

### Occupancy Grid Mapping

### Simultaneous Localization and Mapping

### The GraphSLAM Algorithm

### The Sparse Extended Information Filter

### The FastSLAM Algorithm

## Planning and Control

### Markov Decision Processes

### Partially Observable Markov Decision Processes

### Approximate POMDP Techniques

### Exploration



{% include pseudocode.html id="1" code="
\begin{algorithm}
\caption{Grid-based FastSLAM}
\begin{algorithmic}
\PROCEDURE{FastSLAM}{$X_{t-1}, u_t, z_t$}
    \STATE $\bar{X}_t = X_t = \empty$
    \FOR{$m = 1$ \TO $M$}
        \STATE $x_t^{[k]} = $ \CALL{MotionUpdate}{$u_t, x_{t-1}^{[k]}$}
        \STATE $w_t^{[k]} = $ \CALL{SensorUpdate}{$z_t, x_{t}^{[k]}$}
        \STATE $m_t^{[k]} = $ \CALL{UpdateOccupancyGrid}{$z_t, x_{t}^{[k]}, m_{t-1}^{[k]}$}
        \STATE $\bar{X}_t = \bar{X}_t + \left < x_{t}^{[k]}, w_{t}^{[k]} \right >$
    \ENDFOR
    \FOR{$k = 1$ \TO $M$}
        \STATE draw $i$ with probability $w_t^{[i]}$ 
        \STATE add $\left < x_t^{[i]}, m_t^{[i]} \right >$ \TO $X_t$
    \ENDFOR
    \RETURN $X_t$ 
\ENDPROCEDURE
\end{algorithmic}
\end{algorithm}
" %}


## Links



## Reference

