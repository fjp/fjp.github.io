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

The project shows how the `rqt_turtle` plugin for ROS' [GUI framework rqt](http://wiki.ros.org/rqt) was created. 
The plugin can be used to draw inside [turtlesim](http://wiki.ros.org/turtlesim) with turtlebot.
Although the following description might help you to write your own rqt plugin, also have a look at the official [rqt tutorials](http://wiki.ros.org/rqt/Tutorials). 
There are tutorials explaining how to write plugins using python or C++. The `rqt_turtle` plugin is written in C++.
The [source code](https://github.com/fjp/rqt-turtle) is hosted on GitHub.

## Create Empty ROS Package

The first step is to create an empty ROS package and specify the required dependencies. 
Note, that it is possible to add missing dependencies later on.
Inside a [ros workspace](http://wiki.ros.org/catkin/workspaces) use the [`catkin create`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_create.html) command from [catkin-tools](https://catkin-tools.readthedocs.io/en/latest/) to creat the empty `rqt_turtle` package:

```console
$ catkin create pkg rqt_turtle \
    -a "Franz Pucher" "ros@fjp.at" \
    -m "Franz Pucher" "ros@fjp.at" \
    -l "MIT" \
    -d "rqt plugin for ROS rqt to draw in turtlesim using turtlebot." \
    --catkin-deps roscpp rqt_gui rqt_gui_cpp
```

## Modify package.xml

To make the rqt plugin discoverable, for catkin, you must declare the plugin in the `package.xml` by referencing a `plugin.xml`.

```xml
  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->
    <rqt_gui plugin="${prefix}/plugin.xml"/>
  </export>
```

Now create the `plugin.xml` with the following content:

```xml
<library path="src">
  <class name="Turtle Plugin" type="rqt_turtle::TurtlePlugin" base_class_type="rqt_gui_py::Plugin">
    <description>
      Plugin for ROS rqt to draw in turtlesim using turtlebot.
    </description>
    <qtgui>
      <!-- optional grouping...
      <group>
        <label>Group</label>
      </group>
      <group>
        <label>Subgroup</label>
      </group>
      -->
      <label>Turtle Plugin</label>
      <icon type="theme">system-help</icon>
      <statustip>Plugin for ROS rqt to draw in turtlesim using turtlebot.</statustip>
    </qtgui>
  </class>
</library>
```

A description of these attributes is found in the [rqt tutorial](http://wiki.ros.org/rqt/Tutorials/Create%20your%20new%20rqt%20plugin#Attributes_of_library_element_in_plugin.xml).
