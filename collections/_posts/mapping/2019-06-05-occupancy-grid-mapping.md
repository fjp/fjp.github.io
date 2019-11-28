---
layout: single
title:  "Occupancy Grid Mapping"
date:   2019-06-05 19:00:42 +0200
excerpt: "Occupancy Grid Map algorithm to map an environment."
permalink: /posts/mapping/occupancy-grid-mapping/
categories: [robotics, mapping, grid map, ros]
tags: [robotics, gird, mapping, map, occupancy grid mapping, occupancy, ros]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Kalman filter"
header-includes:
  - \usepackage{algorithm2e}
header:
  image: /assets/posts/2019-04-17-monte-carlo-localization/Images/Step0.png
  teaser: /assets/posts/2019-04-17-monte-carlo-localization/Images/Step0.png
  overlay_image: /assets/posts/2019-04-17-monte-carlo-localization/particle_filters.gif #keep it square 200x200 px is good
gallery_iterations:
  - url: /assets/posts/2019-04-17-monte-carlo-localization/Images/Step0.png
    image_path: /assets/posts/2019-04-17-monte-carlo-localization/Images/Step0.png
    alt: "Iteration 0"
    title: "Iteration 0"
---

This post describes how to map an environment with the Occupancy Grid Map algorithm.
The algorithm can map any arbitrary environment by dividing it into a finite number of grid cells.

## Mapping

In mapping problems the robot pose $x\_{1:t}$ is known and the map $m\_{t}$ at time $t$, either static or dynamic is unknown. 
Therefore the mapping problem is to find the posterior belief of the map $p(m\_t|x\_{1:t}, z\_{1:t})$ given the robot poses and its measurements $z\_{1:t}$.
Compared to [localization](/posts/localization/), where a robot pose is estimated in a known map, the goal of mapping is to estimate the map itself. 

## Motivation

Mapping is required in dynamic as well as static environments. In dynamic environments obstacles in the environment change and the robot has adapt to these changes by updating its map.
Because sensor measurements are always noisy, also static environments need to be mapped. This is a correction to account fo the noise and to obtain a high accuracy map. 

## Challenges

The challenges in mapping are the number of state variables. In localization, only the robots pose is estimated with its $x$ and $y$ position in a known environment, where a map was previously generated. 
A map on the other hand lies in a continuous space. 
This can lead to infinitely many variables used to describe the map. Additional uncertainty is present through sensor data perception. 
Other challenges are the space and its geometries that should be mapped. For example repetitive environments such as walkways with no doors or similar looking ones.  
To map an environment, the robot pose is assumed to be known and the occupancy grid mapping algorithm can be used to solve the problem.
However, the hypotesis space is hughe. This is the space of all possible maps that can be formed during mapping. The space is highly dimensional because maps are defined over a continuous space.
To overcome these challenges, the occupancy grid map algorithm presents a discrete approximation of the map. But even with these approximations the space of all possible maps will still be high.
Therefore, the challenge is to estimate the full posterior map for maps with high dimensional spaces. 

Information about walls and objcts are required to map an environment. This data can be gathered by laser range finders or cameras. With this sensory information a mobile robot can
detect obstacles in the environment. Such data can be combined into a final map. 

The following lists the difficulties while mapping an environment:

1. Environment size: Large environments are harder to map because of the amount of data to be processed. In such a case the robot has to collect all the instantaneous poses and obstacles, form
a resulting map and localize the robot with respect to this map. 
2. Perceptual range: If the map is larger than the perceptual range of the robot the mapping problem becomes more challenging. 
3. Noisy sensors: All sensors, such as perception sensors, odometry sensors and actuators. During mapping the noise has to be filtered from these sensors to avoid adding up these errors.
4. Perceptual ambiguity: Repetitive environments or places that look a like result in an ambiguity. The robot then must correlate between these two places, which the robot travelled to at different points in time.
5. Cycles: If the robot travels in a cyclic manner, for example going back and forth in a corridor. In such a case robot odometry incrementally accumulates an error, which results in a large error at the end of the cycle. 


## Mapping with Known Poses

The problem of generating a map under the assumption that the robot poses are known and non noisy is refered to as mapping with known poses. The problem can be represented as a graph with a node $m$ as the map,
$z_t$, which are nodes representing the measurements of sensing the environment and the poses $x_t$ also as nodes. 
The occupancy grid mapping algorithm can estimate the posterior given noisy measurements and known poses. Usually though the poses are unknown which is the case in [SLAM](/posts/slam/slam). Mapping, however, happens after SLAM.
During SLAM the robot built a map of the environment and localized itself relative to it. After SLAM the occupancy grid mapping algorithm uses the exact robot poses filtered from SLAM.
With the known poses from SLAM and noisy measurements, the mapping algorithm generates a map fit for path planning and navigation. 

## ROS gmapping

The [gmapping](http://wiki.ros.org/gmapping) ROS package uses the Grid-based FastSLAM algorithm. This package contains the single `slam_gmapping` node, which subscribes to the `tf` and `scans` topics. 
Using these inputs, it generates a 2D occupancy grid map and outputs robot poses on the `map` and `entropy` topics. Additional map data is provided through the `map_metadata` topic. Another way to access the map is to use the service provided by the node. 
To demonstrate gmapping, turtlebot will be deployed in the willow garage environment inside gazebo. Moving the turtlebout around using the `teleop` package and running the `slam_gmapping` node will generate a map. 


## Links

The gmapping algorithm can be found [here](https://openslam-org.github.io/gmapping.html).

## Reference

This post is a summary of the lesson on Occupancy Grid Mapping from the Robotics Nanodegree of Udacity found [here](https://eu.udacity.com/course/robotics-software-engineer--nd209).
