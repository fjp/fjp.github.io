---
layout: single #collection
title: ROS rqt plugin for turtlesim
permalink: /ros/rqt-turtle/
excerpt: "Plugin for ROS rqt to draw in turtlesim using turtlebot."
date: 2020-08-23 20:00:35 +0200
categories: [robotics]
tags: [ros, rqt, noetic, plugin, qt5, qt, turtlesim, turtlebot]
comments: true
use_math: true
toc: true
# classes: wide
header:
  #teaser: /assets/collections/diffbot/assembly/board-plate/03-board-plate-front-left.jpg
  #overlay_image: /assets/collections/diffbot/assembly/board-plate/03-board-plate-front-left.jpg
  overlay_filter: 0.5 # same as adding an opacity of 0.5 to a black background
  caption: rqt_turtle
  show_overlay_excerpt: true
sidebar:
  nav: ros
---

The project shows how the rqt_turtle plugin for ROS rqt was created, which can be used to draw with turtlebot in turtlesim.
Although it is also possible to write rqt plugins using python the package uses C++.

The first step is to create an empty ROS package and specify the required dependencies. 
Note, that it is possible to add missing dependencies later on.
Inside a [ros workspace](http://wiki.ros.org/catkin/workspaces) use the following [catkin-tools](https://catkin-tools.readthedocs.io/en/latest/) command to creat the empty `rqt_turtle` package:

```console
catkin create pkg rqt_turtle roscpp rqt_gui rqt_gui_cpp
```
