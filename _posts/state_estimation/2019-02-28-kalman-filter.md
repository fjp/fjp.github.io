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
the control input vector $\mathbf{u}_{k}$ and the input matrix $\mathbf{B}$.

$$
\begin{equation}
\mathbf{x}_{k+1} = \mathbf{F} \mathbf{x}_{k} + \mathbf{B} \mathbf{u}_{k} 
\label{eq:state-transition}
\end{equation}
$$

It is common that not all of the states of a system are observable on the output $\mathbf{z}\_{k}$. This is modeled by the output matrix $\mathbf{H}\_{k}$ which maps the state $x$ to the observation $\mathbf{z}\_{k}$. 
A direct relation of the input $u$ to the output $\mathbf{z}\_{k}$ can be modeled with the matrix $\mathbf{D}\_{k}$, which is usually set to zero. 

$$
\begin{equation}
\mathbf{z}_{k} = \mathbf{H}_{k} \mathbf{x}_{k} + \mathbf{D}_{k} \mathbf{u}_{k} 
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
\begin{equation}
\hat{\mathbf{P}}_{k|k-1} =  \mathbf{F}_{k-1} \hat{\mathbf{P}}_{k-1} \mathbf{F}_{k-1}^{T} + \mathbf{Q}_{k-1}
\end{equation}
$$

### Measurment Update or Correction

When a new measurement $\mathbf{z}\_{k}$ is available, the predicted states are updated using this new information. This reduces the uncertainty while taking into account the uncertainty of the measurement $\mathbf{R}_k$.
To use the additional measurement information $\mathbf{z}\_{k}$, it is compared to the predicted state $\hat{\textbf{x}}\_{k|k-1}$ through the measurement matrix $\textbf{H}\_k$. This results in what's called the residual $\ref{eq:residual}$ also know as innovation.

$$
\begin{equation}
\tilde{\textbf{y}}_k = \textbf{z}_k - \textbf{H}_k\hat{\textbf{x}}_{k|k-1}
\label{eq:residual}
\end{equation}
$$

The covariance of the innovation is defined in $\ref{eq:residual-cov}$.

$$
\begin{equation}
\textbf{S}_k = \textbf{H}_k \hat{\textbf{P}}_{k|k-1} \textbf{H}_k^\text{T} + \textbf{R}_k
\label{eq:residual-cov}
\end{equation}
$$


The posterior state $\ref{eq:posterior-state}$ and its covariance $\ref{eq:posterior-covariance}$ are calculated according to the followign equations.

$$
\begin{equation}
\hat{\textbf{x}}_{k} = \hat{\textbf{x}}_{k|k-1} + \hat{\textbf{K}}_k\tilde{\textbf{y}}_k
\label{eq:posterior-state}
\end{equation}
$$

$$
\begin{equation}
\hat{\textbf{P}}_{k} = \hat{\textbf{P}}_{k|k-1} - \hat{\textbf{K}}_k \textbf{S}_k \hat{\textbf{K}}_k^\text{T}
\label{eq:posterior-covariance}
\end{equation}
$$


These equations make use of the Kalman gain $\ref{eq:kalman-gain}$ which takes into account the ratio of the measurement and state prediciton uncertainty. 

$$
\begin{equation}
\hat{\textbf{K}}_k = \hat{\textbf{P}}_{k|k-1}\textbf{H}_k^\text{T}\textbf{S}_k^{-1}
\label{eq:kalman-gain}
\end{equation}
$$

Using the Kalman gain in $\ref{eq:posterior-covariance}$ results in the following commonly used equation.

$$
\begin{align}
\hat{\textbf{P}}_{k} &= \hat{\textbf{P}}_{k|k-1} - \hat{\textbf{K}}_k \textbf{S}_k \hat{\textbf{K}}_k^\text{T} \\
\hat{\textbf{P}}_{k} &= \hat{\textbf{P}}_{k|k-1} - \hat{\textbf{K}}_k \textbf{S}_k \left( \hat{\textbf{P}}_{k|k-1}\textbf{H}_k^\text{T}\textbf{S}_k^{-1} \right)^\text{T} \\
\hat{\textbf{P}}_{k} &= \hat{\textbf{P}}_{k|k-1} - \hat{\textbf{K}}_k \textbf{S}_k \textbf{S}_k^{-1} \textbf{H}_k \hat{\textbf{P}}_{k|k-1} \\
\hat{\textbf{P}}_{k} &= \hat{\textbf{P}}_{k|k-1} - \hat{\textbf{K}}_k \textbf{H}_k \hat{\textbf{P}}_{k|k-1} \\
\hat{\textbf{P}}_{k} &= \left( \textbf{I} - \hat{\textbf{K}}_k \textbf{H}_k \right) \hat{\textbf{P}}_{k|k-1} \\
\end{align}
$$

## Measuemrent Uncertainty

In the following two sections the influence of the measurement covariance are investigated

### Low Measurement Uncertainty

Assuming a low measurement uncertainty $\textbf{R} = 0$ means that the sensor produces perfectly accurate measurements and results in the following.

$$
\begin{align}
\textbf{S}_k &= \textbf{H}_k \hat{\textbf{P}}_{k|k-1} \textbf{H}_k^\text{T} \\
\hat{\textbf{K}}_k &= \hat{\textbf{P}}_{k|k-1}\textbf{H}_k^\text{T}\textbf{S}_k^{-1} \\
\hat{\textbf{K}}_k &= \hat{\textbf{P}}_{k|k-1}\textbf{H}_k^\text{T} \left( \textbf{H}_k \hat{\textbf{P}}_{k|k-1} \textbf{H}_k^\text{T}  \right)^{-1} \\
\hat{\textbf{K}}_k &= \hat{\textbf{P}}_{k|k-1}\textbf{H}_k^\text{T} \textbf{H}_k^\text{T,-1} \hat{\textbf{P}}_{k|k-1}^{-1} \textbf{H}_k^\text{-1}  \\
\hat{\textbf{K}}_k &= \hat{\textbf{P}}_{k|k-1} \hat{\textbf{P}}_{k|k-1}^{-1} \textbf{H}_k^\text{-1}  \\
\hat{\textbf{K}}_k &= \textbf{H}_k^\text{-1}  \\
\end{align}
$$

The Kalman gain is equal to the inverse measurement matrix. Using this insight and the residual $\ref{eq:residual}$ results in the following updated mean and covariance

$$
\begin{align}
\hat{\textbf{x}}_{k} &= \hat{\textbf{x}}_{k|k-1} + \hat{\textbf{K}}_k\tilde{\textbf{y}}_k \\
\hat{\textbf{x}}_{k} &= \hat{\textbf{x}}_{k|k-1} + \hat{\textbf{K}}_k  \left( \textbf{z}_k - \textbf{H}_k\hat{\textbf{x}}_{k|k-1} \right) \\
\end{align}
$$

Using the previous result that the Kalman gain is equal to the inverse measurment matrix results in

$$
\begin{align}
\hat{\textbf{x}}_{k} &= \hat{\textbf{x}}_{k|k-1} + \textbf{H}_k^\text{-1} \left( \textbf{z}_k - \textbf{H}_k\hat{\textbf{x}}_{k|k-1} \right) \\
\hat{\textbf{x}}_{k} &= \hat{\textbf{x}}_{k|k-1} + \textbf{H}_k^\text{-1} \textbf{z}_k - \textbf{H}_k^\text{-1} \textbf{H}_k\hat{\textbf{x}}_{k|k-1} \\
\hat{\textbf{x}}_{k} &= \hat{\textbf{x}}_{k|k-1} + \textbf{H}_k^\text{-1} \textbf{z}_k - \hat{\textbf{x}}_{k|k-1} \\
\hat{\textbf{x}}_{k} &= \textbf{H}_k^\text{-1} \textbf{z}_k \\
\end{align}
$$

The final equation maps the measurement through the inverse measurement matrix to the state. This shows that a measurement uncertainty of zero results in a new mean which is entirely defined by the measurement.
Any information that the state prediciton provided is not used for the new state because it is not needed. The measuremnt is perfect.

### High Measurement Uncertainty

A high measurement uncertainty $R = \infty$ on the other hand, shows a negible sensor and results in the following


$$
\begin{align}
\textbf{S}_k &= \textbf{H}_k \hat{\textbf{P}}_{k|k-1} \textbf{H}_k^\text{T} + \mathbf{R}_k\\
\textbf{S}_k &= \infty \\
\end{align}
$$

A residual matrix of infinity results in a Kalman gain that is zero

$$
\begin{align}
\hat{\textbf{K}}_k &= \hat{\textbf{P}}_{k|k-1}\textbf{H}_k^\text{T}\textbf{S}_k^{-1} \\
\hat{\textbf{K}}_k &= 0 
\end{align}
$$

Therefore the equation for the new mean is

$$
\begin{align}
\hat{\textbf{x}}_{k} &= \hat{\textbf{x}}_{k|k-1} + \hat{\textbf{K}}_k\tilde{\textbf{y}}_k \\
\hat{\textbf{x}}_{k} &= \hat{\textbf{x}}_{k|k-1} \\
\end{align}
$$


## C++ Implementation

{% highlight cpp %}
#include <iostream>
#include <math.h>
#include <tuple>
#include "Core" // Eigen Library
#include "LU"   // Eigen Library

using namespace std;
using namespace Eigen;

float measurements[3] = { 1, 2, 3 };

tuple<MatrixXf, MatrixXf> kalman_filter(MatrixXf x, MatrixXf P, MatrixXf u, MatrixXf F, MatrixXf H, MatrixXf R, MatrixXf I)
{
    for (int n = 0; n < sizeof(measurements) / sizeof(measurements[0]); n++) {

        // Measurement Update
        MatrixXf Z(1, 1);
        Z << measurements[n];

        MatrixXf y(1, 1);
        y << Z - (H * x);

        MatrixXf S(1, 1);
        S << H * P * H.transpose() + R;

        MatrixXf K(2, 1);
        K << P * H.transpose() * S.inverse();

        x << x + (K * y);

        P << (I - (K * H)) * P;

        // Prediction
        x << (F * x) + u;
        P << F * P * F.transpose();
    }

    return make_tuple(x, P);
}

int main()
{

    MatrixXf x(2, 1);// Initial state (location and velocity) 
    x << 0,
    	 0; 
    MatrixXf P(2, 2);//Initial Uncertainty
    P << 100, 0, 
    	 0, 100; 
    MatrixXf u(2, 1);// External Motion
    u << 0,
    	 0; 
    MatrixXf F(2, 2);//Next State Function
    F << 1, 1,
    	 0, 1; 
    MatrixXf H(1, 2);//Measurement Function
    H << 1,
    	 0; 
    MatrixXf R(1, 1); //Measurement Uncertainty
    R << 1;
    MatrixXf I(2, 2);// Identity Matrix
    I << 1, 0,
    	 0, 1; 

    tie(x, P) = kalman_filter(x, P, u, F, H, R, I);
    cout << "x= " << x << endl;
    cout << "P= " << P << endl;

    return 0;
}
{% endhighlight %}


## Links

* [Kalman and Bayesian Filters in Python](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python)

## References

<a href="https://www.amazon.de/Applied-Optimal-Estimation-Mit-Press/dp/0262570483/ref=as_li_ss_il?s=books-intl-de&ie=UTF8&qid=1551390892&sr=1-1&keywords=optimal+estimation&linkCode=li2&tag=fjp-21&linkId=85bcdf63f00d2b9b918d322eb6079771&language=de_DE" target="_blank"><img border="0" src="//ws-eu.amazon-adsystem.com/widgets/q?_encoding=UTF8&ASIN=0262570483&Format=_SL160_&ID=AsinImage&MarketPlace=DE&ServiceVersion=20070822&WS=1&tag=fjp-21&language=de_DE" ></a><img src="https://ir-de.amazon-adsystem.com/e/ir?t=fjp-21&language=de_DE&l=li2&o=3&a=0262570483" width="1" height="1" border="0" alt="" style="border:none !important; margin:0px !important;" />


<a href="https://www.amazon.de/Probabilistic-Robotics-INTELLIGENT-ROBOTICS-AUTONOMOUS/dp/0262201623/ref=as_li_ss_il?ie=UTF8&qid=1551730012&sr=8-1&keywords=probabilistic+robotics&linkCode=li2&tag=fjp-21&linkId=7fba87448a00855820511f309b7a4d41&language=de_DE" target="_blank"><img border="0" src="//ws-eu.amazon-adsystem.com/widgets/q?_encoding=UTF8&ASIN=0262201623&Format=_SL160_&ID=AsinImage&MarketPlace=DE&ServiceVersion=20070822&WS=1&tag=fjp-21&language=de_DE" >test</a><img src="https://ir-de.amazon-adsystem.com/e/ir?t=fjp-21&language=de_DE&l=li2&o=3&a=0262201623" width="1" height="1" border="0" alt="" style="border:none !important; margin:0px !important;" />
