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
It provides tools for modeling the robot in software, represented by the robot hardware interface (`hardware_interface::RobotHW`), and comes 
with ready to use out of the box controllers, thanks to a common interface ([`controller_interface::ControllerBase`](http://docs.ros.org/melodic/api/controller_interface/html/c++/classcontroller__interface_1_1ControllerBase.html)) for different applications which then talk to third-party tools. 
The important aspects to highlight is that the hardware access to the robot is de-coupled, so you don't expose how you talk to your robot. This means that controllers and your robot hardware structure are decoupled so you can develop them independently and in isolation which enables you to share and mix and match controllers.

When you want to write your robot backend, there's a possibility to leverage an out of the box simulation backend (`gazebo_ros_control` plugin). 
After you've tested your robot in simulation, and want to work on the real robot hardware, you will want to write your custom hardware backend for your robot. For this, there are also tools that help making this task easier for you.

There is already a set of existing controllers (see the [`ros_controllers`](http://wiki.ros.org/ros_controllers) meta package) that are robot agnostic, which should hopefully enable you to leverage some of those. However, if your application requires it, then there are tools ([`controller_interface`](http://wiki.ros.org/controller_interface?distro=melodic)) that help you also implement custom controllers. ROS control also provides the [controller manager](http://wiki.ros.org/controller_manager?distro=melodic) for controlling the life cycle of your controllers.


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
There you find messages and action definitions, tools that help you writing real-time code ([realtime_tools](https://github.com/ros-controls/realtime_tools)) in the context of control, hardware interfaces ([`hardware_interface`](http://wiki.ros.org/hardware_interface) and the [`transmission_interface`](http://wiki.ros.org/transmission_interface) package to describe your robot and the [`controller_manager`](http://wiki.ros.org/controller_manager). The core framework is found in the [`ros_control`](https://github.com/ros-controls/ros_control) package which is compatible with [`ros_controllers`](https://github.com/ros-controls/ros_controllers) that provides a set of robot agnostic controllers.

The [`ros_control`](http://wiki.ros.org/ros_control) meta package is composed of the following individual packages:

- [`control_toolbox`](http://wiki.ros.org/control_toolbox): This package contains common modules (PID and Sine) that can be used by all controllers. 
- [`controller_interface`](http://wiki.ros.org/controller_interface): This package contains the interface base class for existing controllers and to develop new ones that will work with the ROS control framework. 
- [`controller_manager`](http://wiki.ros.org/controller_manager): This package provides the infrastructure to load, unload, start, and stop controllers.
- [`controller_manager_msgs`](http://wiki.ros.org/controller_manager_msgs): This package provides the message and service definitions for the controller manager.
- [`hardware_interface`](http://wiki.ros.org/hardware_interface): This contains the base class for the hardware interfaces 
- [`transmission_interface`](http://wiki.ros.org/transmission_interface): This package contains the interface classes to model transmissions ([simple reducers](http://docs.ros.org/melodic/api/transmission_interface/html/c++/classtransmission__interface_1_1SimpleTransmission.html), [differentials](http://docs.ros.org/melodic/api/transmission_interface/html/c++/classtransmission__interface_1_1DifferentialTransmission.html) or [four-bar linkages](http://docs.ros.org/melodic/api/transmission_interface/html/c++/classtransmission__interface_1_1FourBarLinkageTransmission.html))


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


The hardware interface is a software representation of the robot and its abstract hardware.
In ROS control speak, we have the notion of resources, which are actuators, joints and sensors.  
Some resources are are read-only, such as joint states, IMU, force-torque sensors, and so on, and some are read and write compatible, such as position, velocity, and effort joints.
A bunch of hardware interfaces, where one is just a set of similar resources, represent a robot.

The robot hardware, represented in software, and the controllers are connected via ROS control's interfaces, 
not to be confused with typical ROS interfaces like topics, actions or services. 
Instead, we are just passing pointers around which is real-time safe. The interfaces of the robot hardware are
used by controllers to connect and communicate with the hardware. 
At the leftmost part of the image we see that controllers have their interfaces, which are typically ROS interfaces
(topics, services or actions) that are custom so your controller can expose whatever it wants.


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
    MyRobot(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);
    
    // Talk to HW
    void read();
    void write();
    
    
    // Requirement only if needed ...
    virtual bool checkForConflict(...) const;
}
```

To implement your custom robot hardware interface you inherit from a class called `hardware_interface::RobotHW` that is part of ROS control. In the constructor you set up your robot, either implemented in code (see for example: [eborghi10/my_ROS_mobile_robot](https://github.com/eborghi10/my_ROS_mobile_robot/blob/master/my_robot_base/include/my_robot_hw_interface.h)) or using a robot description in the form of an urdf model. The latter requires that you pass the urdf model to the constructor or load it from the parameter server. An example using the urdf description can be found in the [ros_control_boilerpalte](https://github.com/PickNikRobotics/ros_control_boilerplate) repository.

After the robot setup we need to implement the `read()` and `write()` methods, which is robot specific and up to you to implement. The `read()` method is used to populate the raw data with your robot's state. `write()` is used to send out 
raw commands via CAN bus, Ethernet, Ethercat, serial port or whatever protocol your robot hardware uses.

Within the ROS community, there are some projects that can help you out with this. 
Examples are [ROS-Industrial](https://rosindustrial.org/), [rosserial](http://wiki.ros.org/rosserial), [SR Ronex](http://wiki.ros.org/sr_ronex) and other custom solutions out there that you can leverage. 

In case exclusive resource ownership is not good enough, you can always reimplement the virtual member function [`hardware_interface::RobotHW::checkForConflicts()`](http://docs.ros.org/melodic/api/hardware_interface/html/c++/classhardware__interface_1_1RobotHW.html#ab491cf8d359534fe104f59300c188878) with your custom implementation.

In summary, the package called [hardware_interface](http://wiki.ros.org/hardware_interface) (see also its [C++ API](http://docs.ros.org/melodic/api/hardware_interface/html/c++/annotated.html)) provides all the building blocks for constructing a robot hardware abstraction. It is the software representation of your robot 
and the idea is to abstract hardware away which is done using resources (actuators, joints and sensors), interfaces, that are sets of similar resources, and make up a robot, which is a set of interfaces. The hardware interface also handles resource conflicts and by default we have exclusive resource ownership. 

Out of the box ROS control provides resources and interfaces to read states such as joint state (position, velocity and effort) IMU and force-torque sensors. If, in addition to reading, you also want to send commands there are interfaces for position control joints, but also velocity and effort control joints. For a list of all available interfaces check out the [Hardware Interfaces](http://wiki.ros.org/ros_control#Hardware_Interfaces) section on the `ros_control` wiki page.

Similar interfaces exist for actuators, which will be explained later. 


#### Simulation Backend



<figure>
    <a href="/assets/ros/ros-control/simulation-overview.png"><img src="/assets/ros/ros-control/simulation-overview.png"></a>
    <figcaption>Robot simulation using ROS control and Gazebo (Source: <a href="http://wiki.ros.org/ros_control">ROS.org ros_control</a>).</figcaption>
</figure>

To simulate your robot there is a package called [`gazebo_ros_control`](http://wiki.ros.org/gazebo_ros_control) which is located outside the ROS control organization repositories in [ros-simulation/gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs). It is a [Gazebo](http://gazebosim.org/) plugin for ROS control.
There exists a default plugin, which populates all the joint interfaces from the URDF by parsing transmission specification and optionally joint limits, which will be describe later in the post. If you use the default plugin and want your
robot to show up in your Gazebo simulator, the following is all you have to add to your urdf description, apart from the robot's transmissions.

```xml
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/my_robot</robotNamespace>
  </plugin>
</gazebo>
```

This makes it simple to test ROS control because we don't have to write any robot hardware interface code. 
Instead, we just need to setup some configuration files. 

There's also a custom plugin where most of the details are up to you. TODO source?

### Controllers

In the following images we concentrate on the controllers 

<figure class="half">
  <a href="/assets/ros/ros-control/controllers01.png"><img src="/assets/ros/ros-control/controllers01.png"></a>
    <a href="/assets/ros/ros-control/controllers02.png"><img src="/assets/ros/ros-control/controllers02.png"></a>
    <figcaption>Controllers (Source: <a href="http://wiki.ros.org/ros_control">ROS.org ros_control</a>).</figcaption>
</figure>

and see more of the arm controller's internals.

<figure class="half">
  <a href="/assets/ros/ros-control/arm-controller.png"><img src="/assets/ros/ros-control/arm-controller.png"></a>
    <figcaption>arm controller (Source: <a href="http://wiki.ros.org/ros_control">ROS.org ros_control</a>).</figcaption>
</figure>

As mentioned, a controller requires resources in the form of `ros_control` hardware interfaces (located on the right side in the image). On the left hand side we have the controller's ROS API.

Controllers use a plugin interface that implements the controller lifecycle. The lifecycle is a simple state machine with transitions between two states which are "stopped" and "running". A controller has two computation units where one is real-time safe and the other is non real-time.

The state machine uses non real-time operations to load and unload a controller. 

<figure>
  <a href="/assets/ros/ros-control/controller-load-unload.png"><img src="/assets/ros/ros-control/controller-load-unload.png"></a>
    <figcaption>Plugin interface transitions: load and unload (Source: <a href="http://wiki.ros.org/ros_control">ROS.org ros_control</a>).</figcaption>
</figure>

Loading controller is done by the `controller_manager` which initializes the controller plugin defined in a config yaml file and checks requisites, for example the existence of hardware resources 
(note that this is not the same as hardware resource conflict handling). In the arm controller example, 
we need two joints named `arm_1_joint` and `arm_2_joint` and they must implement the `PositionJointInterface`.
In case your robot doesn't have that interface with the resources, the controller you want to use cannot work with that robot. You can also check for things like the robot URDF description configuration or a controller specific configuration like ROS parameters scoped within the controller namespace. Finally in the load method you setup the ROS interfaces which define how your clients will talk to the controller. 

The steps to unload a controller plugin are implemented in the destructor of the controller.


ROS control has real-time safe operations, for transitioning between the stopped and running states.

<figure>
  <a href="/assets/ros/ros-control/controller-start-stop.png"><img src="/assets/ros/ros-control/controller-start-stop.png"></a>
    <figcaption>Plugin interface transitions: start and stop (Source: <a href="http://wiki.ros.org/ros_control">ROS.org ros_control</a>).</figcaption>
</figure>

When you `start()` the controller the following is what gets executed before the first controller `update()`:

- Resource conflict handling (different from resources existence: by this time we know that resources exists but we check if the resource is already in use or available for your controller (in the case of exclusive ownership)
- The next typical policy is to apply what's called semantic zero. This means, after the controller starts you would for example hold the position, set the velocity to zero, activate gravity compensation or something else that makes sense in your use case.

The `stop()` operation goes from the "running" to the "stop" state. The typical policy for this operation is to cancle goals.
Cancelling all goals avoids a rapid movement the next time you start your controller because it doesn't try to resume what it was doing the last time around.


The last real-time safe operation is `update()` which is a real-time safe computation, if implemented accordingly. This means you have to take care the implementation of `update()` is actually real-time safe, or if you don't have real-time constrains, that the implementation is deterministic. 
The `updated()` method is executed periodically in the "running" state.


Finally, the last part of computations that exist in controllers are non real-time operations which are executed asynchronously and take place in the callbacks of your controller's ROS interface. 

In summary, controllers are dynamically loadable **plugins** that have an interface which defines a very **simple state machine**. This interface clearly separates the operations that are **non real-time safe** from those that are required to be **real-time safe**. Finally, the computation can take place in the **controller update**, which in this case is both periodic and real-time safe, and we have computation in the **ROS API callbacks**, which is asynchronous and non real-time safe.


#### `controller_toolbox`

To write your own controllers there is a helpful package [`controller_toolbox`](http://wiki.ros.org/control_toolbox) which contains tools useful for writing
**controllers** or **robot abstractions**.

The following is a list of tools you can use:

- **Pid** PID loop
- **SineSweep** for joint frequency analysis
- **Dither** white noise generator
- **LimitedProxy** for convergence without overshoot
- ...


#### `ros_controllers`

Another helpful repository is [`ros_controllers`](http://wiki.ros.org/ros_controllers) which provides ready to use out of the box controllers. 
If you plan to develop a controller, you should first check in this repository if it already exists.

This repository has three controllers for reporting the joint state from sensors:

- [`joint_state_controller`](http://wiki.ros.org/joint_state_controller) reads joint states and publishes it to the [`sensor_msgs/JointState`]() topic.
Note: although it contains the word controller in its name, it is not actually controlling anything. However, it should always be used to publish the current joint states for other nodes such as `tf`. It is also worth mentioning that it should not be confused with [`joint_state_publisher`](http://wiki.ros.org/joint_state_publisher) (see [this answer](https://answers.ros.org/question/303358/what-is-the-difference-between-joint_state_publisher-and-joint_state_controller/) for a distinction). 
- [`imu_sensor_controller`](http://wiki.ros.org/imu_sensor_controller) publishes [`sensor_msgs/Imu`](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html) topics.
- [`force_torque_sensor_controller`](http://wiki.ros.org/force_torque_sensor_controller) publishes [`geometry_msgs/Wrench`](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Wrench.html) topics.

It also provides multiple general purpose controllers:

- 

### The Control Loop





## Reference

- [ROSCon 2014 talk](https://vimeo.com/107507546) and the presented [slides](http://roscon.ros.org/2014/wp-content/uploads/2014/07/ros_control_an_overview.pdf)
