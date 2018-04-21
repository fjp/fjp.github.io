---
layout: posts
title: Vehicle Models
date:   2017-12-03 17:19:31 +0100
categories:
  - control
  - model
  - dynamic
tags:
  - udacity
  - model
  - dynamic
  - mpc
use_math: true
---


There are basically two types of vehicle models:

- Kinematic
  - [CTRV model]({{ site.baseurl }}{% post_url self-driving/2017-10-17-ctrv-model %})
- [Dynamic]({{ site.baseurl }}{% post_url self-driving/2017-12-03-dynamic-models %})

## Kinematic Models

Kinematic models are simplifications of dynamic models that ignore tire forces, gravity, and mass.
This simplification reduces the accuracy of the models, but it also makes them more tractable.
At low and moderate speeds, kinematic models often approximate the actual vehicle dynamics.


## Dynamic Models

Dynamic models aim to embody the actual vehicle dynamics as closely as possible.
They might encompass tire forces, longitudinal and lateral forces, inertia, gravity, air resistance, drag, mass, and the geometry of the vehicle.
Not all dynamic models are created equal! Some may consider more of these factors than others.
Advanced dynamic models even take internal vehicle forces into account - for example, how responsive the chassis suspension is.


both model types follow nonholonomic constrains.

## Nonholonomic

In a real vehicle, actuators are limited by the design of the vehicle in fundamental physics.
For example, a vehicle can't have a steering angle of 90 degrees. It's impossible.
Thus, it doesn't make sense for us to even consider these kinds of inputs.
The vocabulary to describe this model constraint is known as [nonholonomic](https://en.wikipedia.org/wiki/Nonholonomic_system), because the vehicle can't move in arbitrary directions.

{% include figure image_path="/assets/posts/2017-12-03-vehicle-models/nonholonomic.png" caption="Nonholonomic model (Source: [Udacity self driving car ND](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/af4fcd4f-eb1f-43d8-82b3-17bb1e71695f/concepts/e0c4c6fd-18e6-45b4-bdb8-867909908119))" %}


It's limited by its steering angle constraints. We can solve this by setting lower and upper bounds for the actuators.
For example, we may set the steering angle to be between minus 30 and 30 degrees, and the acceleration to be between minus one and one.
Minus one signifying full brake, and one signifying full acceleration.

{% include figure image_path="/assets/posts/2017-12-03-vehicle-models/actuator-constraints.png" caption="Actuator constraints forces (Source: [Udacity self driving car ND](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/af4fcd4f-eb1f-43d8-82b3-17bb1e71695f/concepts/e0c4c6fd-18e6-45b4-bdb8-867909908119))" %}


In practice, the upper and lower bounds should mimic the actual vehicle as closely as possible.

## Additional reading

- [Nonholonomic system](https://en.wikipedia.org/wiki/Nonholonomic_system)
