---
layout: single
title:  "Extended Kalman Filter"
date:   2019-03-06 17:31:41 +0200
excerpt: "The Kalman filter is used for state estimation but limited to linear models. To deal with nonlinear models the extended Kalman filter can be used instead."
permalink: /posts/state-estimation/extended-kalman-filter/
categories: [robotics, state estimation, Kalman filter]
tags: [robotics, state estimation, Kalman filter, c++, extended, nonlinear]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Kalman filter"
header:
#  teaser: /assets/images/opencv_logo.png
#  overlay_image: /assets/images/opencv_logo.png #keep it square 200x200 px is good
---


As explained in the [previous post](/posts/state-estimation/extended-kalman-filter), a [Kalman filter](https://de.wikipedia.org/wiki/Kalman-Filter) is used as a state predictor for linear systems.
To deal with nonlinear systems given by nonlinear state prediction $x'=f(x)$ and/or nonlinear measurement function $z = h(x)$ the extended Kalman filter can be used.
The essence of this extended approach is the usage of the Taylor series, to linearize the nonliner system around an operating point.

The extended Kalman filter requires the calculation of the Jacobian of a nonlinear function as part of every single iteration, since the mean (which is the point that at which the nonlinear funciton is linearize about) is updated.

## Differences to the linear Kalman Filter

The Kalman filter consists of an prediction and an measurement update step. In the case of a nonlinear model the state is still updated with the 
nonlinear state equation $x'= f(x)$. However, the covariance cannot be updated using the nonlinear function as it would result in a non-Gaussian distribution. 
To update the state covariance, the nonlinear function $\mathbf{f}(\mathbf{x})$ needs to be linearized around a small section at the operation point $\mathbf{x}$, which results in the Jacobian matrix $\mathbf{F}$.
The linearization is achieved by the taking the first two terms of the Taylor Series of the function centered around the mean. 

$$
\mathbf{F} = \mathbf{f}(\mathbf{\mu}) + \frac{\partial \mathbf{f}(\mathbf{\mu})}{\partial \mathbf{x}} (\mathbf{x} - \mathbf{\mu})
$$


Because the measurement function is assumed to be nonlinear the residual results in the following equation

$$
y = z - h(x')
$$

To update the measurement covariance matrix $\mathbf{S}$, the measuremtn function $\mathbf{h}$ needs to be linearized around the mean.
This is done again with the Taylor series or the Jacobian in the multi-dimensional case. 

$$
\mathbf{h}(\mathbf{x}) \approx \mathbf{h}(\mathbf{\mu}) + (\mathbf{x} - \mathbf{\mu}) J_{\mathbf{h}}(\mathbf{\mu}) 
$$
either the measurement update or the state prediction. 

## Links

* [Kalman and Bayesian Filters in Python](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python)

## References

[Applied Optimal Estimation](https://www.amazon.de/dp/0262570483/?tag=fjp-21), edited by Arthur Gelb (The Analytic Sciences Corporation) (affiliate link)


[Probabilistic Robotics](https://www.amazon.de/dp/0262201623/?tag=fjp-21) by Sebastian Thrun, Wolfram Burgard and Dieter Fox (affiliate link)
