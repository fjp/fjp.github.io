---
layout: single #collection
title: ROS Turtle Pong
permalink: /ros/turtle-pong/
excerpt: "Pong game for ROS turtlesim."
date: 2020-09-02 20:00:35 +0200
categories: [robotics]
tags: [ros, noetic, c++, pong, turtlesim, turtlebot]
comments: true
use_math: true
toc: true
# classes: wide
header:
  #teaser: /assets/collections/diffbot/assembly/board-plate/03-board-plate-front-left.jpg
  #overlay_image: /assets/collections/diffbot/assembly/board-plate/03-board-plate-front-left.jpg
  overlay_filter: 0.5 # same as adding an opacity of 0.5 to a black background
  caption: turtle_pong
  show_overlay_excerpt: true
sidebar:
  nav: ros
---

This video shows the first version of the Pong arcade video game for the Robot Operating System (ROS 1 Noetic) using turtlesim.
The post will explain how the `turtle_pong` package for ROS' [turtlesim](http://wiki.ros.org/turtlesim) was created. 

<iframe width="560" height="315" src="https://www.youtube.com/embed/i83dNyfm_QE" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

This project is part of the Robocademy 
[Robot Operating System Learning Path](https://robocademy.com/2020/06/25/enroll-in-robot-operating-system-learning-path-by-lentin-joseph/) by 
[Lentin Joseph](https://lentinjoseph.com/). 


The [source code](https://github.com/fjp/ros-turtle-pong) is hosted on GitHub.


## Create Empty ROS Package

The first step is to create an empty ROS package and specify the required dependencies. 
Note, that it is possible to add missing dependencies later on.
Inside a [ros workspace](http://wiki.ros.org/catkin/workspaces) use the [`catkin create`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_create.html) command from [catkin-tools](https://catkin-tools.readthedocs.io/en/latest/) to creat the empty `turtle_pong` package:

```console
catkin_ws/src$ catkin create pkg turtle_pong \
    -a "Franz Pucher" "ros@fjp.at" \
    -m "Franz Pucher" "ros@fjp.at" \
    -l "MIT" \
    -d "Pong game for ROS turtlesim." \
    --catkin-deps roscpp
```


The game is made up of three nodes:

- `ball` a turtle that acts as the ball and contains logic to bounce off walls.
- `key` the turtles representing the paddles which are controlled with the keyboard to move them up and down.
- `game_logic` node that keeps track of the game state.

The `ball` class subscribes to three poses, the ball itself and the two players. 

Note: To use class methods as callbacks see the wiki page [Tutorials/UsingClassMethodsAsCallbacks](http://wiki.ros.org/roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks).
