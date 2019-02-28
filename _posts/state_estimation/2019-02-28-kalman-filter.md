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

Note: This article is about the linear Kalman filter that assumes a linear model. Other versions of the Kalman filter such as 
the extended Kalman filter and the unscented Kalman filter are used for nonlinear models. 

## State Space Representation

In the state space representation a system is described by two linear equations. 
The state transition equation $\ref{eq:state-transition}$ that consists of the state vector $x$ itself, which ought to be estimated, the state transition matrix $\mathbf{F}$,
the control input vector $vec{u}_{k}$ and the input matrix $\mathbf{B}$.

$$
\begin{equation}
vec{x}_{k+1} = \mathbf{F} vec{x}_{k} + \mathbf{B} vec{u}_{k} 
\label{eq:state-transition}
\end{equation}
$$

It is common that not all of the states of a system are observable on the output $z$. This is modeled by the output matrix $\mathbf{H}$ which maps the state $x$ to the observation $z$. 
A direct relation of the input $u$ to the output $z$ can be modeled with the matrix $\mathbf{D}$, which is usually set to zero. 

$$
\begin{equation}
vec{z}_{k+1} = \mathbf{H} vec{x}_{k+1} + \mathbf{D} vec{u}_{k} 
\label{eq:measurement-equation}
\end{equation}
$$

## Multidimensional Kalman Filter

The following sections and equations describe the multidimensional Kalman filter which relies on a multidimensional Gaussian probability distribution.
The mean of the Gaussian can be seen as the value of a state, whereas the standard deviation or variance shows the uncertainty of the state value. 
A high variance implies hiher uncertainty about the state, where on the other hand a narrow Gaussian has a low variance an therefore a more certain mean value. 

### State Transition or Prediction

In the first step of the filter loop, the previous state estimation, known as prior, is used to predict the current state. This leads to the following mean value.

$$
\hat{\mathbf{x}}_{k|k-1} = \mathbf{F}_{k-1}\hat{\mathbf{x}}_{k-1} + \mathbf{B}_{k-1}\mathbf{u}_{k-1}
$$

The covariance is multiplied by the state transtion matrix squared and further increased by the uncertainty of the prediction with the matrix $\mathbf{Q}$.

$$
\hat{\mathbf{P}}_{k|k-1} =  \mathbf{F}_{k-1} \hat{\mathbf{P}}_{k-1} \mathbf{F}_{k-1}^{\text{T}} + \mathbf{Q}_{k-1}
$$

### Measurment Update or Correction

When a new measurement $z_{k}$ is available, the predicted states are updated using this new information. This reduces the uncertainty while taking into account the uncertainty of the measurement $\mathbf{R}_k$.

$$
\tilde{\textbf{y}}_k = \textbf{z}_k - \textbf{H}_k\hat{\textbf{x}}_{k|k-1}
$$

$$
\hat{\textbf{P}}_{k} = \hat{\textbf{P}}_{k|k-1} - \hat{\textbf{K}}_k \textbf{S}_k \hat{\textbf{K}}_k^\text{T}
$$

$$
\hat{\textbf{x}}_{k} = \hat{\textbf{x}}_{k|k-1} + \hat{\textbf{K}}_k\tilde{\textbf{y}}_k
$$

$$
\textbf{S}_k = \textbf{H}_k \hat{\textbf{P}}_{k|k-1} \textbf{H}_k^\text{T} + \textbf{R}_k
$$

Kalman gain

$$
\hat{\textbf{K}}_k = \hat{\textbf{P}}_{k|k-1}\textbf{H}_k^\text{T}\textbf{S}_k^{-1}
$$


## References

<a href="https://www.amazon.de/Applied-Optimal-Estimation-Mit-Press/dp/0262570483/ref=as_li_ss_il?s=books-intl-de&ie=UTF8&qid=1551390892&sr=1-1&keywords=optimal+estimation&linkCode=li2&tag=fjp-21&linkId=85bcdf63f00d2b9b918d322eb6079771&language=de_DE" target="_blank"><img border="0" src="//ws-eu.amazon-adsystem.com/widgets/q?_encoding=UTF8&ASIN=0262570483&Format=_SL160_&ID=AsinImage&MarketPlace=DE&ServiceVersion=20070822&WS=1&tag=fjp-21&language=de_DE" ></a><img src="https://ir-de.amazon-adsystem.com/e/ir?t=fjp-21&language=de_DE&l=li2&o=3&a=0262570483" width="1" height="1" border="0" alt="" style="border:none !important; margin:0px !important;" />

