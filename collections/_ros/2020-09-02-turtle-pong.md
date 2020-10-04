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

## Usage

To use the `turtle_pong` package clone this repository into the `src` folder of your catkin workspace:

```console
fjp@ubuntu:/home/fjp/catkin_ws/src$ git clone https://github.com/fjp/ros-turtle-pong.git
```

Then build the workspace with `catking-tools` or `catkin_make` and source the new package:

```console
# catkin-tools:
fjp@ubuntu:/home/fjp/catkin_ws$ catkin build
# or use
fjp@ubuntu:/home/fjp/catkin_ws$ catkin_make
# source your workspace using the setup.bash or setup.zsh depening on your shell
fjp@ubuntu:/home/fjp/catkin_ws$ source devel/setup.bash
fjp@ubuntu:/home/fjp/catkin_ws$ source devel/setup.zsh
```

Finally start `roscore`, run `turtlesim` and `pong.launch`:

```console
roscore
rosrun turtlesim turtlesim_node
roslaunch turtle_pong pong.launch
```

Note that each of the three commands above should be executed from another terminal so that it will run in its own process.

The game can be played with the w/s keys and the up/down arrow keys to control the left and right player (turtle) respectively.

The next sections explain how the package was created and how its working.

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

## Class Architecture

The game is made up of three nodes:

- `ball` a turtle that acts as the ball and contains logic to bounce off walls.
- `key` the turtles representing the paddles which are controlled with the keyboard to move them up and down.
- `pong` node that spawns the three turtles and keeps track of the game state.

<figure>
    <a href="https://raw.githubusercontent.com/fjp/ros-turtle-pong/master/docs/rosgraph.svg?sanitize=true"><img src="https://raw.githubusercontent.com/fjp/ros-turtle-pong/master/docs/rosgraph.svg?sanitize=true"></a>
    <figcaption>ROS Computation Graph.</figcaption>
</figure>



The `ball` class subscribes to three poses, the ball itself and the two players. 

Note: To use class methods as callbacks see the wiki page [Tutorials/UsingClassMethodsAsCallbacks](http://wiki.ros.org/roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks).

<figure>
    <a href="https://raw.githubusercontent.com/fjp/ros-turtle-pong/master/docs/rosgraph.svg?token=AAJ2DWQ6BPTFUPV44S6L6S27LNYM4?sanitize=true"><img src="https://raw.githubusercontent.com/fjp/ros-turtle-pong/master/docs/rosgraph.svg?token=AAJ2DWQ6BPTFUPV44S6L6S27LNYM4?sanitize=true"></a>
    <figcaption>ROS Computation Graph.</figcaption>
</figure>

## References

- [http://wiki.ros.org/roscpp/Overview/Publishers and Subscribers](http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers)
- [Tutorials/UsingClassMethodsAsCallbacks](http://wiki.ros.org/roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks)
