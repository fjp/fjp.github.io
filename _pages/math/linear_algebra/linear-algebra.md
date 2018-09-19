---
layout: single #collection
title: Mathematics - Linear Algebra
permalink: /math/linear-algebra
excerpt: "Awesome list on mathematical topics"
date: 2018-08-25 15:41:35 +0200
categories: [math, linear algebra]
tags: [math, linear algebra, vectors]
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
  #teaser: /assets/projects/autonomous-rc-car/hpi-racing-bmw-m3_thumb.png
  #overlay_image: /assets/projects/autonomous-rc-car/hpi-racing-bmw-m3.png
redirect_from:
    - /math/
---

## Introduction

The following sections introduce some topics of linear algebra. For more mathematical topics checkout other [![Awesome](https://awesome.re/badge.svg)](/math/) list.


## Vectors

<iframe width="560" height="315" src="https://www.youtube.com/embed/fNk_zzaMoSs?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>



## Span, Bases and Linear Combinations

<p>
The span of two vectors $\vec{v}$ and $\vec{w}$ is the set of all their linear combinations.

$$
a\vec{v}+b\vec{w}
$$

where $a$ and $b$ varay over all real numbers $\mathbb{R}$.
</p>
{: .notice}

<p>
Vectors are said to be linearly independent if

$$
\vec{u} \neq a\vec{v}+b\vec{w}
$$

for all values $a$ and $b$.

Otherwise they are linearly dependent if one vector can be expressed in terms of the others
$$
\vec{u} = a\vec{v}+b\vec{w}
$$
</p>
{: .notice}

<p>
The basis of a vector space is a set of linearly independent vectors that span the full space
</p>
{: .notice}

<iframe width="560" height="315" src="https://www.youtube.com/embed/k7RM-ot2NWY?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

## Matrices as Linear Transformations

Important topic in linear algebra that shows the relation between linear transformations (can be seen as a funciton $f(x)$) and matrices.
Usually such linear transformations transform one vector $\vec{v}$ to another vector $\vec{w} = L(\vec{v})$.

Given a basis with vectors $\hat{i} = \begin{bmatrix}1 & 0 \end{bmatrix}$ and $\hat{j} = \begin{bmatrix}0 & 1 \end{bmatrix}$ that span the 2D space,
matrices can be thought of as transformations of that space.
In a given matrix

$$
\begin{bmatrix}
a & c \\
b & d
\end{bmatrix}
$$

with respect to the original basis, the first matrix column tells us where the first basis vector $\hat{i}$ lands (or gets transformed to)
and the second column shows where the basis vector $\vec{j}$ lands in that linear transformation.

<p>
A linear transformation is linear if it has two properties:

- All lines must remain lines (without getting curved)
- Origin must remain fixed in place

which means that grid lines must remain parallel and evenly spaced.
</p>
{: .notice}

<p>
To program a transformation function numerically, it is only necessary to record where the basis vectors $\hat{i}$ and $\hat{j}$ each land.
With this information, other vectors can be transformed using these transformed basis vectors as columns in a new transformation matrix.
An arbitrary vector $\vec{v}$ can then be transformed by that matrix. This can be seen as scaling the transformed basis vectors by the
vector components of $\vec{v}$ or as one specific linear combination.

$$
\begin{bmatrix}
i_x & j_x \\
i_y & j_y
\end{bmatrix}
\begin{bmatrix}
v_x \\
v_y
\end{bmatrix} =
\underbrace{
v_x
\begin{bmatrix}
i_x \\
i_y
\end{bmatrix} +
v_y
\begin{bmatrix}
j_x \\
j_y
\end{bmatrix}}_{\text{Where all the intuition is}}
=
\begin{bmatrix}
i_x v_x & j_x v_y \\
i_y v_x & j_y v_y
\end{bmatrix}
$$
</p>
{: .notice}

<iframe width="560" height="315" src="https://www.youtube.com/embed/kYB8IZa5AuE?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

## Matrix Multiplication as Composition


<iframe width="560" height="315" src="https://www.youtube.com/embed/XkY2DOUCWMU?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

## Three Dimensional Linear Transformations

Transformations in three dimensions work just like transformations in two dimensions only with three dimensional vectors as input and output and three by three
transformation matrices.

<p>
$$
\vec{v} =
\begin{bmatrix}
x_i \\
y_i \\
z_i \\
\end{bmatrix} \rightarrow
L(\vec{v}) \rightarrow
\begin{bmatrix}
x_o \\
y_o \\
z_o \\
\end{bmatrix} = \vec{w}
$$
</p>
{: .notice}

As in two dimensions, a three dimensional transformation is completely described by where the three unit basis vectors $\hat{i}$, $\hat{j}$, $\hat{k}$ go. Given a three dimensional transformation matrix, the output coordinates of the basis vectors are given by the matrix columns.
Transforming a three dimensional input vector can be thought of scaling each basis vector with the corresponding input coordinate, which results in
the transformed output vector.

<p>
$$
\vec{w} = L(\vec{v}) =  
\begin{bmatrix}
\vert  & \vert & \vert \\
\hat{i} & \hat{j} & \hat{k} \\
\vert  & \vert  & \vert \\
\end{bmatrix}
\begin{bmatrix}
x \\
y \\
z \\
\end{bmatrix} =
x \hat{i} + y \hat{j} + z \hat{k}
$$
</p>
{: .notice}

<iframe width="560" height="315" src="https://www.youtube.com/embed/rHLEWRxRGiM?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>


## The Determinant


<iframe width="936" height="527" src="https://www.youtube.com/embed/Ip3X9LOh2dk?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

## Inverse Matrices, Column Space and Null Space


## Dot Products and Cross Products


## Change of Basis
