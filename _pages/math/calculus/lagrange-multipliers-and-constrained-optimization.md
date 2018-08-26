---
layout: single #collection
title: Mathematics - Lagrange Multipliers and Constrained Optimization
permalink: /math/optimization-lagrange-multipliers
excerpt: "lagrange multipliers and constrained optimization"
date: 2018-08-25 15:41:35 +0200
categories: [math, calculus, optimization]
tags: [math, calculus, optimization, constrained]
comments: true
use_math: true
toc: true
# toc_label: "Unscented Kalman Filter"
classes: wide
header:
  #overlay_image: /assets/projects/autonomous-rc-car/hpi-racing-bmw-m3.png
  #overlay_filter: 0.5 # same as adding an opacity of 0.5 to a black background
  #caption: "Source: [**hpiracing**](http://www.hpiracing.com/de/kit/114343)"
  #show_overlay_excerpt: true
  teaser: https://upload.wikimedia.org/wikipedia/commons/5/55/LagrangeMultipliers3D.png
  overlay_image: https://upload.wikimedia.org/wikipedia/commons/5/55/LagrangeMultipliers3D.png
redirect_from:
    - /math/
---

## Introduction

The following section is about constrained optimization and lagrange multipliers.
For more mathematical topics checkout other [![Awesome](https://awesome.re/badge.svg)](/math/) list.

## Prerequisites

- [Contour maps](https://www.khanacademy.org/math/multivariable-calculus/applications-of-multivariable-derivatives/constrained-optimization/a/g/a/contour-maps)
- [Gradient](https://www.khanacademy.org/math/multivariable-calculus/applications-of-multivariable-derivatives/constrained-optimization/a/g/a/the-gradient)
- [Local maxima and minima](https://www.khanacademy.org/math/multivariable-calculus/applications-of-multivariable-derivatives/constrained-optimization/a/g/a/maximums-minimums-and-saddle-points)


## Lagrange multipliers Introduction

The Lagrange multiplier technique lets you find the maximum or minimum of a multivariable function $f(x, y, \dots)$
when there is some constraint $\color{red}{g(x,y,\dots)}$ on the input values $x$, $y$, $\dots$ you are allowed to use.


<figure class="half">
    <a href="https://upload.wikimedia.org/wikipedia/commons/5/55/LagrangeMultipliers3D.png"><img src="https://upload.wikimedia.org/wikipedia/commons/5/55/LagrangeMultipliers3D.png"></a>
    <a href="https://upload.wikimedia.org/wikipedia/commons/5/55/LagrangeMultipliers2D.png"><img src="https://upload.wikimedia.org/wikipedia/commons/b/bf/LagrangeMultipliers2D.svg"></a>
    <figcaption>Multivariable function $\color{blue}{f(x, y)}$ and constraint $\color{red}{g(x,y) = c}$.</figcaption>
</figure>


the gradient of $f$ evaluated at a point $(x_0, y_0)$ always gives a vector perpendicular to the contour line passing through that point.
This means when the contour lines of two functions $\color{blue}f$ and $\color{red}g$ are tangent, their gradient vectors are parallel.
This tangency means their gradient vectors align:

<p>
<strong>Tangency Condition</strong>

$$
\label{tangency}
\nabla \color{blue}{f(x_0,y_0)} = \color{green}{\lambda_0} \nabla \color{red}{g(x_0,y_0)}
$$
</p>
{: .notice}

Here, $\color{green}{\lambda}$ represents some constant. Some authors use a negative constant, $-\color{green}{\lambda}$.

Let's see what this looks like in our example where $\color{blue}{f(x,y)=2x+y}$ and $\color{red}{g(x,y)=x^2+y^2}$.
The gradient of $f$ is

$$
\nabla f(x,y) =
\begin{bmatrix}
\frac{\partial}{\partial x}(2x+y) \\
\frac{\partial}{\partial y}(2x+y)
\end{bmatrix} =
\begin{bmatrix}
2 \\
1
\end{bmatrix}
$$

and the gradient of $g$ is

$$
\nabla g(x,y) =
\begin{bmatrix}
\frac{\partial}{\partial x}(x^2+y^2) \\
\frac{\partial}{\partial y}(x^2+y^2)
\end{bmatrix} =
\begin{bmatrix}
2x \\
2y
\end{bmatrix}
$$

Therefore, the tangency condition from equation $\ref{tangency}$ ends up looking like this:

$$
\begin{bmatrix}
2 \\
2
\end{bmatrix} =
\lambda
\begin{bmatrix}
2x \\
2y
\end{bmatrix}
$$

In the example from above, this leads to the following three equations and two three unknowns $x_0$, $y_0$ and $\lambda_0$.

$$
\color{red}{x_0^2 + y_0^2 = 1} \\
$$

$$
\begin{align}
2 &= 2\color{green}{\lambda_0} x_0 \Rightarrow x_0 = \frac{1}{\lambda_0}\\
1 &= 2\color{green}{\lambda_0} y_0 \Rightarrow y_0 = \frac{1}{2\lambda_0} \\
\end{align}
$$

Plugging $x_0$ and $y_0$ into the third equation results in a quadratic equation with two solutions for $\lambda_0$

$$
\lambda_0 = \pm\frac{\sqrt{5}}{2}
$$

This leads to two solutions for the input pair $(x_0,y_0)$ which can be plugged into the original function $f(x,y)$.
One input leads to the minimum and the other to the maximum value of $f$.


In general the we can write these conditions by saying we are looking for constants $x_0$, $y_0$ and $\lambda_0$
that satisfy the constraint and the tangency condition \ref{tangency}.

[Joseph Louis Lagrange](https://en.wikipedia.org/wiki/Joseph-Louis_Lagrange) wrote down a special new function which takes in all the same input variables as $f$ and $g$,
along with the new kid in town $\lambda$, thought of now as a variable rather than a constant.

<p>
<strong>The Lagrangian</strong>

$$
\mathcal{L}(x,y,\dots,\color{green}{\lambda}) = \color{blue}{f(x,y,\dots)} - \color{green}{\lambda}(\color{red}{g(x,y,\dots) - c})
$$

with the Lagrange multiplier $\color{green}{\lambda}$.
</p>
{: .notice}


## References

- [Khan Academy](https://www.khanacademy.org/math/multivariable-calculus/applications-of-multivariable-derivatives/constrained-optimization/a/lagrange-multipliers-single-constraint)
- [Wikipedia](https://en.wikipedia.org/wiki/Lagrange_multiplier)
