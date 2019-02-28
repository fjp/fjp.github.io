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


A Kalman filter is used as a state predictor for a system with a model given in [state-space representation](https://en.wikipedia.org/wiki/State-space_representation).
In this representation a system is described by two linear equations. 

Note: This article is about the linear Kalman filter that assumes a linear model. Other versions of the Kalman filter such as 
the extended Kalman filter and the unscented Kalman filter are used for nonlinear models. 

The state transition equation $\ref{eq:state-transition}$ that consists of the state vector $x$ itself, which ought to be estimated, the state transition matrix $\mathbf{F}$,
the control input vector $u_{k}$ and the input matrix $\mathbf{B}$.

$$
\begin{equation}
x_{k+1} = \mathbf{F} x_{k} + \mathbf{B} u_{k} 
\label{eq:state-transition}
\end{equation}
$$

The following sections and equations describe the multidimensional Kalman filter which relies on a multidimensional Gaussian probability distribution.
The mean of the Gaussian can be seen as the value of a state, whereas the standard deviation or variance shows the uncertainty of the state value. 
A high variance implies hiher uncertainty about the state, where on the other hand a narrow Gaussian has a low variance an therefore a more certain mean value. 




