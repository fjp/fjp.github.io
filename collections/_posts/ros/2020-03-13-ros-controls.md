---
layout: single
title:  "ROS Control, An overview"
date:   2020-03-13 19:31:41 +0200
excerpt: "ROS Control combines a set of packages that include controller interfaces, controller managers, transmissions and hardware_interfaces."
permalink: /posts/ros/ros-control/
categories: [robotics, control, controllers, ros, ros-control]
tags: [robotics, control, controllers, ros, ros-control, c++, hardware, interface]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Kalman filter"
header:
  teaser: /assets/collections/2wd-robot/ros/ros_control/gazebo_ros_control.png
  overlay_image: /assets/collections/2wd-robot/ros/ros_control/gazebo_ros_control.png #keep it square 200x200 px is good
  overlay_filter: 0.5 # same as adding an opacity of 0.5 to a black background
  caption: "Source: [**ROS Control Wiki**](http://wiki.ros.org/ros_control)"
  show_overlay_excerpt: true
---


ROS Control combines a set of packages that include controller interfaces, controller managers, transmissions and hardware_interfaces.


## Big Picture and Goals

Let's start with a typical scenario: you have a robot, or even more general, a piece of hardware, and you would 
like to run an application that leverages ROS specific libraries like the Navigation stack or MovieIt!. 
All these libraries are well tested and you don't want to rewrite them. On the other hand you probably have a 
computer that is running a simulator like Gazebo to simulate your robot. To make ends meet we can make use of the ROS control project.

<figure>
    <a href="/assets/ros/ros-control/ros-control-overview.png"><img src="/assets/ros/ros-control/ros-control-overview.png"></a>
    <figcaption>Overview of ROS control project (Source: <a href="http://wiki.ros.org/ros_control">ROS.org ros_control</a>).</figcaption>
</figure>


ROS control helps you writing controllers including real-time constraints, transmissions and joint limits. 
It provides tools for modeling the robotic in software, represented by the robot hardware interface (`hardware_interface::RobotHW`), and comes 
with ready to use out of the box controllers for different applications which then talk to third-party tools. 
The important aspects to highlight is that the hardware access to the robot is de-coupled, so we you don't expose how we talk to our robot. This means that controllers and your robot hardware structure are decoupled so you can develop them independently and in isolation which enables you to share and mix and match controllers.

When you want to write your robot backend, there's a possibility to leverage an out of the box simulation backend (`gazebo_ros_control` plugin). 
After you've tested your robot in simulation, and want to work on the real robot hardware, you will want to write your custom hardware backend for your robot. For this, there are also tools that help making this task easier for you.

There is already a set of existing controllers that are robot agnostic, which should hopefully enable you to leverage some of those. However, if your application requires it, then there are tools (controller interface) that help you also implement custom controllers. ROS control also provides the controller manager for controlling the life cycle of your controllers.


When it comes to third party libraries, there is out of the box compatibility with general purpose robot tools like visualization, `tf` and `rqt` plugins. The project provides also compatibility with the navigation stack for differential drive robots and biped humanoid robots and it also enables motion planning with `MoveIt`. 

The whole idea is that what you can focus on your actual application or research paper and not have to bother writing the boiler plate code to get everything working with your robot hardware. 

Another important aspect is that ROS control is real-time ready in the sense that if your application has (hard) real-time constraints then you can use ROS control with it. However, if your application doesn't have real-time constraints then it is not imposed on you. Also the choice of using a real-time operating system is entirely up to you. 

In a nutshell, the goals of the project are to 

- lower the entry barrier for exposing hardware to ROS. 
- promote the reuse of control code in the same spirit that ROS has done for non-control code, at least non-real-time control code. 
- provide a set of ready-to-use tools. 
- have a real-time ready implementation.

### History

By the year 2009 there was the `pr2_controller_manager` that was developed by Willow Garage which was specific for the pr2 robot. This changed in the late 2012, where hiDOF in collaboration with Willow Garage started the ROS control project which by early 2013 was continued by PAL robotics and community efforts. ROS control is basically a robot agnostic version of the `pr2_controller_manager`. 

### Related Work

There's quite a few component-based frameworks like Orocos RTT (and also there's a couple of projects that build upon Orocos RTT as well), openRTM, Yarp. For computational graph models there's Ecto there's Microblx, and many non-robotics implementations, especially within the audio and graphics communities. 

Without ROS control there would be no end-to-end solution for ROS users. ROS control tries to bring standard ROS interfaces (topics, services, actions) closer to the hardware and therefore to push the integration effort to the driver level, the rightmost part of the overview image. 

<figure>
    <a href="/assets/ros/ros-control/ros-control-black-box.png"><img src="/assets/ros/ros-control/ros-control-black-box.png"></a>
    <figcaption>Overview of ROS control project (Source: <a href="http://wiki.ros.org/ros_control">ROS.org ros_control</a>).</figcaption>
</figure>


## Ros Control and its Repositories

The ROS control project consists of a github organization that's called [ros controls](https://github.com/ros-controls).
There you find things from messages and action definitions to tools that help you writing real-time code ([realtime_tools](https://github.com/ros-controls/realtime_tools)) that is useful in the context of control. The core framework is found in the [`ros_control`](https://github.com/ros-controls/ros_control) package which is compatible with [`ros_controllers`](https://github.com/ros-controls/ros_controllers). This package provides a set of robot agnostic controllers.

### Setting up a Robot

First ROS control is split into two parts. First there is the robot hardware abstraction, 
which is used to communicate with the hardware. It provides resources like joints and it also handles resource conflicts. 
On the other hand, we have controllers. They don't talk directly to hardware and they require resources that are provided by the hardware abstraction. 

<figure>
    <a href="/assets/ros/ros-control/controllers-vs-hardware-abstraction.png"><img src="/assets/ros/ros-control/controllers-vs-hardware-abstraction.png"></a>
    <figcaption>Controllers vs Hardware abstraction (Source: <a href="http://wiki.ros.org/ros_control">ROS.org ros_control</a>).</figcaption>
</figure>


#### Hardware Abstraction and Controllers

Let's get a more detailed view of the hardware abstraction and the communication between and with ROS controllers.

<figure>
    <a href="/assets/ros/ros-control/controllers-and-hardware-interfaces.png"><img src="/assets/ros/ros-control/controllers-and-hardware-interfaces.png"></a>
    <figcaption>Controllers and hardware interfaces (Source: <a href="http://wiki.ros.org/ros_control">ROS.org ros_control</a>).</figcaption>
</figure>

In ROS control speak, we have the notion of resources, for example a velocity controlled wheel. Then there are hardware interfaces, where one is just a set of similar resources, and if you pile up a bunch of interfaces you get a robot. 
There are connectors between the hardware interface and the controllers which are raw control interfaces, 
not to be confused with typical raw interfaces like topics, actions or services. 
Instead, we are just passing pointers around which is real-time safe. The interfaces of the robot hardware are
use by controllers to connect and communicate with the hardware. 
At the leftmost part of the image we see that controllers have their interfaces, which are typically ROS interfaces,
such as topics, services or actions, that are custom so your controller can expose whatever it wants.


#### Exclusive Resource Ownwership

You can have multiple controllers accessing the same interface. 
In the image below you can see that the two arm controllers both use the first arm joint. 

<figure>
    <a href="/assets/ros/ros-control/exclusive-resource-ownership.png"><img src="/assets/ros/ros-control/exclusive-resource-ownership.png"></a>
    <figcaption>Exclusive resource ownership (Source: <a href="http://wiki.ros.org/ros_control">ROS.org ros_control</a>).</figcaption>
</figure>

By default there's a policy of exclusive resource ownership. 
In this case, you can either have one or the other but not both controllers running at the same time,
which is enforced by ROS control. If you want other policies, you can implement them as well.

#### Memory Layout

Now we will learn how the hardware interface actually communicates with the real or simulated hardware using doubles and floats which represent the actual data that you want to pass around.

The following image depicts the memory layout of your raw data and you can see the memory regions where you
actually read and write from and to hardware.

<figure class="half">
  <a href="/assets/ros/ros-control/memory-layout-read.png"><img src="/assets/ros/ros-control/memory-layout-read.png"></a>
    <a href="/assets/ros/ros-control/memory-layout-rw.png"><img src="/assets/ros/ros-control/memory-layout-rw.png"></a>
    <figcaption>Memory layouts for a joint state interface for reading states of a resource and another joint command interface for reading and writing from and to a resource (Source: <a href="http://wiki.ros.org/ros_control">ROS.org ros_control</a>).</figcaption>
</figure>

A resource is nothing else than pointers to the raw data that adds some semantics to the data such as describing that it is a position or velocity, the name of the joint and that it belongs for example to a joint state interface. 

As you see in the memory layout image above, joint state interfaces are used for reading joint state.
If you have a resource that can receive commands and at the same time provide feedback you can use the joint command interface. This interface is the base interface for other joint interfaces for position, velocity and effort (see the [C++ `hardware_interface` API](http://docs.ros.org/melodic/api/hardware_interface/html/c++/classhardware__interface_1_1JointCommandInterface.html)).
These interfaces provide semantic meaning which allow you to read the joint state and also send semantically meaningful commands.

The following is a simple code example. 

```c++
#include <hardware_interface/robot_hw.h>

class MyRobot :
    public hardware_interface::RobotHW
{
public:
    MyRobot(); // Setup robot
    
    // Talk to HW
    void read();
    void write();
    
    
    // Requirement only if needed ...
    virtual bool checkForConflict(...) const;
}
```


### Controllers

### The Control Loop





## Reference

- [ROSCon 2014 talk](https://vimeo.com/107507546) and the presented [slides](http://roscon.ros.org/2014/wp-content/uploads/2014/07/ros_control_an_overview.pdf)
