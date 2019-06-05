---
layout: single
title:  "Occupancy Grid Mapping"
date:   2019-06-05 19:00:42 +0200
excerpt: "For Simultaneous Localization and Mapping a lot of algorithms exist. This post shows the basics of SLAM and how Grid-based FastSLAM works using ROS."
permalink: /posts/mapping/occupancy-grid-mapping/
categories: [robotics, mapping, grid map]
tags: [robotics, gird, mapping, map, occupancy grid mapping, occupancy]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Kalman filter"
header-includes:
  - \usepackage{algorithm2e}
header:
  teaser: /assets/posts/2019-04-17-monte-carlo-localization/Images/Step0.png
  overlay_image: /assets/posts/2019-04-17-monte-carlo-localization/particle_filters.gif #keep it square 200x200 px is good
gallery_iterations:
  - url: /assets/posts/2019-04-17-monte-carlo-localization/Images/Step0.png
    image_path: /assets/posts/2019-04-17-monte-carlo-localization/Images/Step0.png
    alt: "Iteration 0"
    title: "Iteration 0"
---

# Mapping

In mapping problems the robot pose $x\_{1:t}$ is known and the map $m\_{t}$ at time $t$, either static or dynamic is unknown. 
Therefore the mapping problem is to find the posterior belief of the map $p(m\_t|x\_{1:t}, z\_{1:t})$ given the robot poses and its measurements $z\_{1:t}$.

The challenges in mapping are the number of state variables. In localization, only the robots pose is estimated with its $x$ and $y$ position. A map on the other hand lies in a continuous space. 
This can lead to infinitely many variables used to describe the map. Additional uncertainty is present through sensor data perception. 
Other challenges are the space and its geometries that should be mapped. For example repetitive environments such as walkways with no doors or similar looking ones.  


The mapping algorithm that is described in this post is occupancy grid mapping. The algorithm can map any arbitrary environment by dividing it into a finite number of grid cells. 

## ROS gmapping

The [gmapping](http://wiki.ros.org/gmapping) ROS package uses the Grid-based FastSLAM algorithm. This package contains the single `slam_gmapping` node, which subscribes to the `tf` and `scans` topics. 
Using these inputs, it generates a 2D occupancy grid map and outputs robot poses on the `map` and `entropy` topics. Additional map data is provided through the `map_metadata` topic. Another way to access the map is to use the service provided by the node. 
To demonstrate gmapping, turtlebot will be deployed in the willow garage environment inside gazebo. Moving the turtlebout around using the `teleop` package and running the `slam_gmapping` node will generate a map. 


## Links

The gmapping algorithm can be found [here](https://openslam-org.github.io/gmapping.html).

## Reference

This post is a summary of the lesson on Occupancy Grid Mapping from the Robotics Nanodegree of Udacity found [here](https://eu.udacity.com/course/robotics-software-engineer--nd209).
