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
  overlay_image: /assets/posts/2019-03-06-ros-kalman-filter/turtlebot_gazebo.png #keep it square 200x200 px is good
gallery_gazebo:
  - url: /assets/posts/2019-03-06-ros-kalman-filter/turtlebot_gazebo.png
    image_path: /assets/posts/2019-03-06-ros-kalman-filter/turtlebot_gazebo.png
    alt: "Gazebo simulation of the turtlebot package."
    title: "Gazebo simulation of the turtlebot package."
gallery_turtlebot:
  - url: /assets/posts/2019-03-06-ros-kalman-filter/turtlebot_rqt_graph.png
    image_path: /assets/posts/2019-03-06-ros-kalman-filter/turtlebot_rqt_graph.png
    alt: "The rqt_graph of the turtlebot package."
    title: "The rqt_graph of the turtlebot package."
gallery_turtlebot_ekf:
  - url: /assets/posts/2019-03-06-ros-kalman-filter/turtlebot_ekf_rqt_graph.png
    image_path: /assets/posts/2019-03-06-ros-kalman-filter/turtlebot_ekf_rqt_graph.png
    alt: "The rqt_graph of the turtlebot and ekf package."
    title: "The rqt_graph of the turtlebot and ekf package."
gallery_turtlebot_ekf_trajectory:
  - url: /assets/posts/2019-03-06-ros-kalman-filter/turtlebot_ekf_trajectory_rqt_graph.png
    image_path: /assets/posts/2019-03-06-ros-kalman-filter/turtlebot_ekf_trajectory_rqt_graph.png
    alt: "The rqt_graph of the turtlebot ekf and trajectory package."
    title: "The rqt_graph of the turtlebot ekf and trajectory package."
gallery_rviz:
  - url: /assets/posts/2019-03-06-ros-kalman-filter/rviz01_add_robot_model.png
    image_path: /assets/posts/2019-03-06-ros-kalman-filter/rviz01_add_robot_model.png
    alt: "Add robot model in rviz"
    title: "Add robot model"
  - url: /assets/posts/2019-03-06-ros-kalman-filter/rviz02_add_camera.png
    image_path: /assets/posts/2019-03-06-ros-kalman-filter/rviz02_add_camera.png
    alt: "Add camera of robot in rviz"
    title: "Add the camera of the robot"
  - url: /assets/posts/2019-03-06-ros-kalman-filter/rviz03_add_odom_topic.png
    image_path: /assets/posts/2019-03-06-ros-kalman-filter/rviz03_add_odom_topic.png
    alt: "Add odom topic to rviz"
    title: "Add the odom topic"
  - url: /assets/posts/2019-03-06-ros-kalman-filter/rviz04_add_ekf_topic.png
    image_path: /assets/posts/2019-03-06-ros-kalman-filter/rviz04_add_ekf_topic.png
    alt: "Add ekf topic to rviz"
    title: "Add the ekf topic"
gallery_rviz_trajectories:
  - url: /assets/posts/2019-03-06-ros-kalman-filter/rviz_trajectories.png
    image_path: /assets/posts/2019-03-06-ros-kalman-filter/rviz_trajectories.png
    alt: "Trajectories of the robot shown in rviz when it is driven around."
    title: "Trajectories of the robot shown in rviz when it is driven around."
gallery_multiplot:
  - url:
    image_path:
    alt: ""
    title: ""
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

The same is done to find the orientation, by integrating the angular velocity provided by the imu.  


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

In the next sections the single packages are explained. To follow the instructions, a ROS [catkin workspace](http://wiki.ros.org/catkin/workspaces) needs to be initialized, which allows to hold the packages.

{% highlight bash %}
mkdir -p /home/workspace/catkin_ws/src
cd /home/workspace/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
{% endhighlight %}

The [`catkin_make`](http://wiki.ros.org/catkin/commands/catkin_make) command is a convenience tool for working with catkin workspaces. Running it the first time in your workspace, it will create a `CMakeLists.txt` symlink in your `src` folder.
This link points to a template `CMakeLists.txt` ROS provides. Calling `catkin_make` will also build any packages located in `~/catkin_ws/src`. 

### TurtleBot Gazebo Package

[TurtleBot](https://www.turtlebot.com/) is not only a ROS package but also a real robot which is used extensively in ROS for localization, mapping and path planning. At the time of writing there exist three different versions of the TurtleBot:

1. The TurtleBot1 with an iRobot Create Base (2010)
2. TurtleBot2 with the Kobuki Base (2012) 
3. TurtleBot3 family (2017)

The TurtleBot platforms follow the [REP 119 specification](http://www.ros.org/reps/rep-0119.html) which is part of the [ROS Enhancement Proposals (REP)](http://www.ros.org/reps/rep-0000.html)
Here, TurtleBot2 is deployed in a gazebo environment and estimate its pose. This is done by using two of its onboard sensors, which consist of rotary encoders, imu and a rgbd camera. 

<iframe width="1280" height="720" src="https://www.youtube.com/embed/MOEjL8JDvd0" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

To obtain the list of subscriber and publisher topics we obtain the turtlebot package from ROS with the following terminal commands. This will clone the package into the catkin workspace

**Clone the package**

New packages from external repositories such as github are conventionally placed into the src folder of your catkin workspace.

{% highlight bash %}
cd /home/workspace/catkin_ws/src
git clone https://github.com/turtlebot/turtlebot_simulator
{% endhighlight %}


**Install the dependencies**

Dependencies for a ROS package can be installed using [`rosdep`](http://docs.ros.org/independent/api/rosdep/html/) which is the dependency tool of ROS.

{% highlight bash %}
cd /home/workspace/catkin_ws
source devel/setup.bash
rosdep -i install turtlebot_gazeboit clone https://github.com/turtlebot/turtlebot_simulator
{% endhighlight %}

Output

{% highlight bash %}
#All required rosdeps installed successfully
{% endhighlight %}


{: .notice--info}
The source command is required to overlay this workspace on top of your environment. To understand more about this see the general [catkin documentation](http://wiki.ros.org/catkin).

**Build package and source the workspace**

{% highlight bash %}
catkin_make
source devel/setup.bash
{% endhighlight %}

**Launch the nodes** 

{% highlight bash %}
roslaunch turtlebot_gazebo turtlebot_world.launch
{% endhighlight %}

{% include gallery id="gallery_gazebo" caption="Gazebo simulation of the turtlebot package." %}

**List the topics**

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

{% include gallery id="gallery_turtlebot" caption="The rqt_graph of the turtlebot package." %}

The turtlebot_gazebo node has the following subscribers and publishers that are important for us:

| Subscribers | /cmd_vel_mux/input/teleop | |
| Publishers  | /odom         | /camera/depth/image_raw | /mobile_base/sensors/imu_data


### Robot Pose EKF Package

In this section we implement the ekf Kalman filter package to localize the robot's pose. 
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

These modifications can be achieved by editing the ekf launch file `robot_pose_ekf.launch` in the `src` folder of the ekf package in order to turn off the 3D odometry.

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


**Build the package**


{% highlight bash %}
cd /home/workspace/catkin_ws
catkin_make
source devel/setup.bash
{% endhighlight %}


**Launch the Node**

{% highlight bash %}
roslaunch robot_pose_ekf robot_pose_ekf.launch
{% endhighlight %}


**Visualize the topics:**

To confirm that the topics of the robot_pose_ekf package and the turtlebot_gazebo package are communicating with each other we check the rqt_graph.

{% highlight bash %}
rosrun rqt_graph rqt_graph
{% endhighlight %}

{% include gallery id="gallery_turtlebot_ekf" caption="The rqt_graph of the turtlebot and ekf package." %}

### Odometry to Trajectory Package

Currently the turtlebot is publishing the unfiltered pose through the `/odom` topic and the ekf is publishing the filtered pose via the `/robot_pose_ekf/odom_combined` topic as the following commands show.

{% highlight bash %}
root@pc:/home/workspace/catkin_ws# rostopic info /odom
Type: nav_msgs/Odometry

Publishers: 
 * /gazebo (http://pc:36747/)

Subscribers: 
 * /robot_pose_ekf (http://pc:44537/)


root@pc:/home/workspace/catkin_ws# rostopic info /robot_pose_ekf/odom_combined 
Type: geometry_msgs/PoseWithCovarianceStamped

Publishers: 
 * /robot_pose_ekf (http://d7ea9f1e67d3:44537/)

Subscribers: None
{% endhighlight %}

To compare filtered and unfiltered trajectories it is required to append the timestamped poses, which are generated over time, to a trajectory using another node.
To do this, two trajectories are created using the `odometry_to_trajectory` ROS package. It subscribes to odometry values and publishes trajectories.  
The package contains two nodes, where the first node subscribes to the unfilterd robot poses and appends the last 1000 poses to a trajectory that is then published.
The second node subscribes to the filtered robot pose from the ekf package and publishes also a trajectory using the last 1000 poses.


**Install the package**

{% highlight bash %}
cd /home/workspace/catkin_ws/src
git clone https://github.com/udacity/odom_to_trajectory
{% endhighlight %}


**Build the package**

{% highlight bash %}
cd /home/workspace/catkin_ws
catkin_make
source devel/setup.bash
{% endhighlight %}


**Launch the Node**

{% highlight bash %}
roslaunch robot_pose_ekf robot_pose_ekf.launch
{% endhighlight %}


**Visualize the topics:**

To confirm that the topics of the robot_pose_ekf package and the turtlebot_gazebo package are communicating with each other we check the rqt_graph.

{% highlight bash %}
rosrun rqt_graph rqt_graph
{% endhighlight %}


{% include gallery id="gallery_turtlebot_ekf_trajectory" caption="The rqt_graph of the turtlebot ekf and trajectory package." %}

When we show only the active nodes and topics, we see now that the /robot_pose_ekf/odom_combined topic is publishing to the subscribing /path_plotter_ekf node. 

### TurtleBot Teleop Package

Currently the robot is standing still which results in trajectories with zero values and the robot is only performing sensor updates without motion updates. 
To get more interesting trajectory data from the previously mentioned node, the robot needs to move and collect further sensory information.
To move the robot we use the [turtlebot_teleop](http://wiki.ros.org/turtlebot_teleop) package, which lets you control a robot using the keyboard or a joystick by publishing driving commands. 

The properties of this package does not subscribe to a topic. It only publishes control commands to the `/cmd_vel_mux/input/teleop` topic. 
This topic is the same to which the turtlebot_gazebo package is subscribing.

To let the turtlebot_gazebo package subscribe to the turtlebot_teleop package via the `/cmd_vel_mux/input/teleop` topic, the new node needs to be launched.
This makes it possible to drive the robot around in the gazebo environment and create trajectories which can be compared. 


**Install the package**

{% highlight bash %}
cd /home/workspace/catkin_ws/src
git clone https://github.com/turtlebot/turtlebot
{% endhighlight %}

**Install the dependencies**

{% highlight bash %}
cd /home/workspace/catkin_ws
source devel/setup.bash
rosdep -i install turtlebot_teleop
{% endhighlight %}


**Build the package**

{% highlight bash %}
cd /home/workspace/catkin_ws
catkin_make
source devel/setup.bash
{% endhighlight %}

**Launch the Node**

{% highlight bash %}
roslaunch turtlebot_teleop keyboard_teleop.launch
{% endhighlight %}

Now it is possible to move the robot around using the keyboard commands.
However, make sure that the terminal running the teleop node is active and receiving the keyboard commands. 

{% highlight bash %}
Control Your Turtlebot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly
{% endhighlight %}

### Rviz Package

[Rviz](http://wiki.ros.org/rviz) is used to visualize the robot data and helpful for debugging. An introduction can be found in the [user guide](http://wiki.ros.org/rviz/UserGuide).
To display the data of the nodes that are running we run rviz.

{% highlight bash %}
rosrun rviz rviz
{% endhighlight %}

After rviz runs, its configuration needs to be configured.

- Change the [Fixed Frame]() to `base_footprint`
- Change the [Reference Frame]() to `odom`
- Add a robot model
- Add a camera and select `/camera/rgb/image_raw` topic
- Add a `/ekfpath` topic and change the display name to EKFPath
- Add a `/odom` topic and change the display name to OdomPath and its color to red `(255,0,0)`


{% include gallery id="gallery_rviz" caption="Add these to the rviz configuration." %}

Completing these steps will result in the following rviz configuration when the robot is controlled by the turtlebot_teleop node.

{% include gallery id="gallery_rviz_trajectories" alt="Trajectories of the robot shown in rviz when it is driven around." caption="Trajectories of the robot shown in rviz when it is driven around." %}

Save this rviz configuration to avoid repeating the steps above.
You can save it for example in the `src` folder of your catkin workspace as `TurtleBotEKF.rviz`.

To test if the saved configuration works, kill the rviz terminal and relaunch rviz with the following parameter.

{% highlight bash %}
rosrun rviz rviz -d /home/workspace/catkin_ws/src/TurtleBotEKF.rviz
{% endhighlight %}

It is also possible to use a launch file in order to run this rviz configuration using `roslaunch`.

{% highlight xml %}
<launch>
  <!--RVIZ-->
  <node pkg="rviz" type="rviz" name="rviz" args="-d /home/workspace/catkin_ws/src/TurtleBotEKF.rviz"/>
</launch>
{% endhighlight %}


## Main Launch

The created launch file can be integrated together with the launch files of the other nodes in a main launch file `main.launch`. 
This simplifies the process of launching all the created nodes in the mentioned packages. Instead of launch them individually in seperate terminals it is sufficient to lauch the main launch file. 

To achieve this, we create a main package that contains the `main.launch`.
A new ROS package can be created with the [`catkin_create_pkg`](http://wiki.ros.org/ROS/Tutorials/catkin/CreatingPackage) command. 

**Create a main package**

{% highlight bash %}
/home/workspace/catkin_ws/src
catkin_create_pkg main
{% endhighlight %}


**Build the package**

{% highlight bash %}
/home/workspace/catkin_ws
catkin_make 
{% endhighlight %}

**Create and edit the main.launch file**

{% highlight bash %}
cd /home/workspace/catkin_ws/src/main
mkdir launch
cd launch
vim main.launch
{% endhighlight %}

Use the content for the `main.lauch` file from [Udacity's GitHub repository](https://github.com/udacity/RoboND-EKFLab/blob/master/main/launch/main.launch).

**Launch the main.launch file**

{% highlight bash %}
cd /home/workspace/catkin_ws/
source devel/setup.bash
roslaunch main main.launch
{% endhighlight %}

## Rqt Multiplot



## Links

- [TurtleBot](https://www.turtlebot.com/)
- [TurtleBot on ROS](http://wiki.ros.org/Robots/TurtleBot)
- [TurtleBot on GitHub](https://github.com/turtlebot)
