---
layout: single
title:  "Monte Carlo Localization"
date:   2019-04-17 19:00:42 +0200
excerpt: ""
permalink: /posts/localization/mcl/
categories: [robotics, localization, monte carlo localization]
tags: [robotics, localization, monte carlo, mcl]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Kalman filter"
header:
  teaser: /assets/posts/2019-03-06-ros-kalman-filter/turtlebot.png
  overlay_image: /assets/posts/2019-03-06-ros-kalman-filter/turtlebot_gazebo.png #keep it square 200x200 px is good
gallery_iterations:
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
gallery_multiplot_traj:
  - url: /assets/posts/2019-03-06-ros-kalman-filter/multiplot_traj.png
    image_path: /assets/posts/2019-03-06-ros-kalman-filter/multiplot_traj.png
    alt: "Plots of the unfiltered and filtered y over x positions from the odometry and ekf."
    title: "Plots of the unfiltered and filtered y over x positions from the odometry and ekf."
---

This post is a summary of the Udacity Lab on localization using [Monte Carlo Localization](https://en.wikipedia.org/wiki/Monte_Carlo_localization) (MCL).
The Udacity repo can be found [here](https://github.com/udacity/RoboND-MCL-Lab)

To follow this tutorial, clone the repo to a folder of your choice.

{% highlight bash %}
git clone https://github.com/udacity/RoboND-MCL-Lab 
{% endhighlight %}


Further details about MCL are found in the [paper](http://robots.stanford.edu/papers/thrun.robust-mcl.pdf) of Sebastian Thrun et al. 

## Monte Carlo Localization Algorithm 

$$
\begin{align}
v(t_1) &= v(t_0) + \int_{t_0}^{t_1} a(t) dt \\  
x(t_1) &= x(t_0) + \int_{t_0}^{t_1} v(t) dt \\
\end{align}
$$


## C++ Implementation

The following headers are used in the lab, which are mainly from the standard c++ library.
One exception is the third party plotting library found [here](https://github.com/lava/matplotlib-cpp) that uses python's matplotlib as its backend.


{% highlight cpp %}
#include "src/matplotlibcpp.h" //Graph Library
#include <iostream>
#include <string>
#include <math.h>
#include <stdexcept> // throw errors
#include <random> //C++ 11 Random Numbers
#include <vector>

namespace plt = matplotlibcpp;
using namespace std;
{% endhighlight cpp %}

Next, some global variables are defined for the fixed landmarks and the world size.
The random generator gets initialized and a forward declaration of two functions is made, namely
`mod` and `gen_real_random`.

{% highlight cpp %}
// Landmarks
double landmarks[8][2] = { { 20.0, 20.0 }, { 20.0, 80.0 }, { 20.0, 50.0 },
    { 50.0, 20.0 }, { 50.0, 80.0 }, { 80.0, 80.0 },
    { 80.0, 20.0 }, { 80.0, 50.0 } };

// Map size in meters
double world_size = 100.0;

// Random Generators
random_device rd;
mt19937 gen(rd());

// Global Functions
double mod(double first_term, double second_term);
double gen_real_random();
{% endhighlight cpp %}

The lab uses a robot class that initializes a robot with a random x and y location and orientation in its constructor.

{% highlight cpp %}
Robot()
{
    // Constructor
    x = gen_real_random() * world_size; // robot's x coordinate
    y = gen_real_random() * world_size; // robot's y coordinate
    orient = gen_real_random() * 2.0 * M_PI; // robot's orientation

    forward_noise = 0.0; //noise of the forward movement
    turn_noise = 0.0; //noise of the turn
    sense_noise = 0.0; //noise of the sensing
}
{% endhighlight %}



{% highlight cpp %}
void set(double new_x, double new_y, double new_orient)
{
    // Set robot new position and orientation
    if (new_x < 0 || new_x >= world_size)
        throw std::invalid_argument("X coordinate out of bound");
    if (new_y < 0 || new_y >= world_size)
        throw std::invalid_argument("Y coordinate out of bound");
    if (new_orient < 0 || new_orient >= 2 * M_PI)
        throw std::invalid_argument("Orientation must be in [0..2pi]");

    x = new_x;
    y = new_y;
    orient = new_orient;
}
{% endhighlight %}


{% highlight cpp %}
{% endhighlight cpp %}

{% highlight cpp %}
{% endhighlight cpp %}

{% highlight cpp %}
{% endhighlight cpp %}

## Results

{% include gallery id="gallery_turtlebot" caption="The rqt_graph of the turtlebot package." %}

## Links


## Reference

This post is a summary of the MCLLab from the Robotics Nanodegree of Udacity found [here](https://eu.udacity.com/course/robotics-software-engineer--nd209)
