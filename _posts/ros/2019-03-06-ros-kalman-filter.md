---
layout: single
title:  "ROS Kalman Filter for Sensor Fusion"
date:   2019-03-06 19:31:41 +0200
excerpt: "The Kalman filter is used for state estimation and sensor fusion. This post shows how sensor fusion is done using the Kalman filter and ROS."
permalink: /posts/ros/ros-kalman-filter/
categories: [robotics, state estimation, Kalman filter, ros]
tags: [robotics, state estimation, Kalman filter, c++, ros]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Kalman filter"
header:
  teaser: /assets/posts/2019-03-06-ros-kalman-filter/turtlebot.png
  overlay_image: /assets/posts/2019-03-06-ros-kalman-filter/turtlebot.png #keep it square 200x200 px is good
---


The [previous post](/posts/state-estimation/extended-kalman-filter) described the extended [Kalman filter](https://de.wikipedia.org/wiki/Kalman-Filter).
This post explains how to create a ROS package that implements an extended Kalman filter, which can be used for sensor fusion.
The sensor data that will be fused together comes from a robots [inertial measurement unit](/projects/autonomous-rc-car/minimu9v5/
/) (imu), rotary encoders (wheel odometry) and vision sensors (camera). The ekf package that is developed in this post will be used to compare the sensor data and
apply sensor fusion to estimate the pose of the robot as it moves around. 
To achieve this, the following packages are used.

- **turtlebot_gazebo** launches a mobile robot in the gazebo environment.
- **robot_pose_ekf** estimates the position and orientation of the robot.
- **odom_to_trajectory** append the odometry values generated over time into a trajectory path.
- **turtlebot_teleop** allows to steer the robot using keyboard commands.
- **rviz** lets us visualize the estimated position and orientation of the robot. 


This post refers to the ROS [documentation](http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers) and makes use of the [publisher](http://docs.ros.org/api/roscpp/html/classros_1_1Publisher.html) and [subscriber](http://docs.ros.org/api/roscpp/html/classros_1_1Subscriber.html) [nodes](http://wiki.ros.org/Nodes) which publish and subscribe [messages](http://wiki.ros.org/Messages) to [topics](http://wiki.ros.org/Topics) of ROS. It explains how to change the mentioned packages to integrate these nodes into a single launch file.  
With this launch file it is possible to launch the entire environment. 

## Mobile Robot Sensors 

The following section starts off with some basics about the sensors of the mobil robot and shows that each has its disadvantages.
To account for these, the sensor readings of each sensor are combined using sensor fusion with the EKF. This is done by taking the noisy measurements,
compare them, filter the noise, removie the uncertainties as much as possible and provide a good estimate of the robot's pose. 

### Inertial Measurement Unit

An intertial measurement unit (imu) usually consists of a gyroscope (3DOF) and a accelerometer (3DOF) and can have a magnetometer (3DOF) as well. 
Each of these elements of the imu has three degrees of fredom (3DOF) which results in 9DOF in total. 
The imu can measure the linear velocity and the position using the accelerometer by integration 

$$
\begin{align}
v(t_1) &= v(t_0) + \int_{t_0}^{t_1} a(t) dt \\  
x(t_1) &= x(t_0) + \int_{t_0}^{t_1} v(t) dt \\
\end{align}
$$

Taking the double integral results in a noisy position. This error is accumulated even more over time and makes it necessary to check the drift or error parameters over time. 

and the angular velocity by integration


### Rotary Encoders

This type of sensors are attached to the robots actuated wheels to measure the velocity and position of the wheels. To estimate the robots position, the integral of the velocity is calculated.
However, the robot wheel might be stuck between obstacles or slip on the ground because the robot might be stuck against a wall. In this case the position won't be calculated correctly.
It is furthermore important to check an encoders resolution. An encoder with low resolution is highly sensitive to slippage. 

### Vision Sensor

A vision sensor can be an rgb 2D camera or even an rgbd camera that can sense the depth of its environment.
The robot is then able to senese the depth towards an obstacle which can be translated to a position. 
The drawbacks of cameras are present in low light environments which would require other sensors, such as radar or lidar. It is also important to look at the field of view (FOV) of the vision system and 
take the smalles range of operation into account.

## The ROS Packages

In the next sections the single packages are explained. To follow the instructions, a ROS catkin workspace needs to be initialized, which allows to hold the packages.

{% highlight bash %}
mkdir -p /home/workspace/catkin_ws/src
cd /home/workspace/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
{% endhighlight %}

### TurtleBot Gazebo Package

[TurtleBot](https://www.turtlebot.com/) is not only a ROS package but also a real robot which is used extensively in ROS for localization, mapping and path planning. At the time of writing there exist three different versions of the TurtleBot:

1. The TurtleBot1 with an iRobot Create Base (2010)
2. TurtleBot2 with the Kobuki Base (2012) 
3. TurtleBot3 family (2017)

The TurtleBot platforms follow the [REP 119 specification](http://www.ros.org/reps/rep-0119.html) which is part of the [ROS Enhancement Proposals (REP)](http://www.ros.org/reps/rep-0000.html)
Here, TurtleBot2 is deployed in a gazebo environment and estimate its pose. This is done by using two of its onboard sensors, which consist of rotary encoders, imu and a rgbd camera. 

<iframe width="1280" height="720" src="https://www.youtube.com/embed/MOEjL8JDvd0" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

To obtain the list of subscriber and publisher topics we obtain the turtlebot package from ROS with the following terminal commands. This will clone the package into the catkin workspace

*Clone the package*

{% highlight bash %}
cd /home/workspace/catkin_ws/src
git clone https://github.com/turtlebot/turtlebot_simulator
{% endhighlight %}


*Install the dependencies*



{% highlight bash %}
cd /home/workspace/catkin_ws
source devel/setup.bash
rosdep -i install turtlebot_gazeboit clone https://github.com/turtlebot/turtlebot_simulator
{% endhighlight %}

Output

{% highlight bash %}
#All required rosdeps installed successfully
{% endhighlight %}


*Build package and source the workspace*

{% highlight bash %}
catkin_make
source devel/setup.bash
{% endhighlight %}

*Launch the nodes* 

{% highlight bash %}
roslaunch turtlebot_gazebo turtlebot_world.launch
{% endhighlight %}

<figure>
  <img src="/assets/posts/2019-03-06-ros-kalman-filter/turtlebot_gazebo.png" alt="Turtlebot in the gazebo simulator">
  <figcaption>Gazebo simulation of the turtlebot package.</figcaption>
</figure>

*List the topics*

{% highlight bash %}
rostopic list
{% endhighlight %}

The topics that will be listed are

{% highlight bash %}
/camera/depth/camera_info
/camera/depth/image_raw
/camera/depth/points
/camera/parameter_descriptions
/camera/parameter_updates
/camera/rgb/camera_info
/camera/rgb/image_raw
/camera/rgb/image_raw/compressed
/camera/rgb/image_raw/compressed/parameter_descriptions
/camera/rgb/image_raw/compressed/parameter_updates
/camera/rgb/image_raw/compressedDepth
/camera/rgb/image_raw/compressedDepth/parameter_descriptions
/camera/rgb/image_raw/compressedDepth/parameter_updates
/camera/rgb/image_raw/theora
/camera/rgb/image_raw/theora/parameter_descriptions
/camera/rgb/image_raw/theora/parameter_updates
/clock
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
/joint_states
/mobile_base/commands/motor_power
/mobile_base/commands/reset_odometry
/mobile_base/commands/velocity
/mobile_base/events/bumper
/mobile_base/events/cliff
/mobile_base/sensors/imu_data
/odom
/rosout
/rosout_agg
/tf
{% endhighlight %}


It is also possible to use rqt graph to visualize the topics of the package.

{% highlight bash %}
rosrun rqt_graph rqt_graph
{% endhighlight %}

<figure>
  <img src="/assets/posts/2019-03-06-ros-kalman-filter/turtlebot_rqt_graph.png" alt="The rqt_graph of the turtlebot package">
  <figcaption>The rqt_graph of the turtlebot package.</figcaption>
</figure>


### Robot Pose EKF Package

The documentation of the [robot_pose_ekf package](http://wiki.ros.org/robot_pose_ekf) shows that the node subscribes to the rotary encoder data through the /odom topic. 
It is also subscribing to the imu data through the /imu_data topic. Lastly the node is subscribing to three dimensional odometry data through 
the /vo topic. These sensor data input gets fused by the ekf which results in a filtered and more accurate output pose than one sensor could provide. 
This combined output is published by ekf node which acts as a publisher. The topic is called /robot_pose_ekf/odom_combined, which is an estimated 3D pose of the robot. 

To install the package its git reposityor is cloned into the src directory of the catkin workspace.

{% highlight bash %}
cd /home/workspace/catkin_ws/src/
git clone https://github.com/udacity/robot_pose_ekf
{% endhighlight %}


To interface the robot_pose_ekf package with the turtlebot_gazebo package and estimate the robots pose it is necessary to rename the following topics to match them.
The turtlebot_gazebo package provides a rgbd topic which cannot direclty be used by the robot_pose_ekf package. Also the 3D odometry /vo topic of the ekf package cannot be provided by the turtlebot_gazebo package.
Therefore we use and match only the remaining two topics:

| turtlebot_gazebo | robot_pose_ekf |
|-------|--------|
| /odom | /odom |
| /mobile_base/sensors/imu_data | /imu_data |

These modifications can be achieved by modifying the ekf launch file robot_pose_ekf.launch in the src folder of the ekf package in order to turn off the 3D odometry.

{% highlight xml %}
<launch>

<node pkg="robot_pose_ekf" type="robot_pose_ekf" name="robot_pose_ekf">
  <param name="output_frame" value="odom_combined"/>
  <param name="base_footprint_frame" value="base_footprint"/>
  <param name="freq" value="30.0"/>
  <param name="sensor_timeout" value="1.0"/>  
  <param name="odom_used" value="true"/>
  <param name="imu_used" value="true"/>
  <param name="vo_used" value="false"/>

  <remap from="imu_data" to="/mobile_base/sensors/imu_data" />    
</node>

</launch>
{% endhighlight %}


*Build the package*


{% highlight xml %}
cd /home/workspace/catkin_ws
catkin_make
source devel/setup.bash
{% endhighlight %}


*Launch the Node*

{% highlight xml %}
roslaunch robot_pose_ekf robot_pose_ekf.launch
{% endhighlight %}


*Visualize the topics:*

To confirm that the topics of the robot_pose_ekf package and the turtlebot_gazebo package are communicating with each other we check the rqt_graph.

{% highlight xml %}
rosrun rqt_graph rqt_graph
{% endhighlight %}


### Odometry to Trajectory Package

### TurtleBot Teleop Package

### Rviz Package




## Links

- [TurtleBot](https://www.turtlebot.com/)
- [TurtleBot on ROS](http://wiki.ros.org/Robots/TurtleBot)
- [TurtleBot on GitHub](https://github.com/turtlebot)
