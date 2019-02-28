---
layout: single
title:  "Kalman Filter"
date:   2019-02-28 17:31:41 +0200
excerpt: "The Kalman filter is used for state estimation and sensor fustion. This post explains it."
permalink: /posts/state-estimation/kalman-filter/
categories: [robotics, state estimation, Kalman filter]
tags: [robotics, state estimation, Kalman filter, c++]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Kalman filter"
header:
#  teaser: /assets/images/opencv_logo.png
#  overlay_image: /assets/images/opencv_logo.png #keep it square 200x200 px is good
---



# Kalman Filter 

A Kalman filter is used as a state predictor for a system with a model given in [state-space representation](https://en.wikipedia.org/wiki/State-space_representation).

$$
x_{k+1} = \mathbf{F} x_{k} + \mathbf{B} u_{k} 
$$

The following sections and equations describe the multidimensional Kalman filter which relies on a multidimensional Gaussian probability distribution.


