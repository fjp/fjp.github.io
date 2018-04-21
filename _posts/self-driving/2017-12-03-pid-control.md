---
layout: posts
title:  "PID Control"
date:   2017-12-03 14:24:31 +0100
categories:
  - control
  - pid
tags:
  - udacity
  - pid control
use_math: true
---


Summary of the PID controller invented 1922 by [Nicolas Minorsky](https://en.wikipedia.org/wiki/Nicolas_Minorsky).

- P is the proportinal gain of the controller which penalizes deviations from the center lane, also known as cross track error (cte).
- I is the integral gain and is used to reduce the steady offset errors. This part is used to reduce past errors caused by unmodeled disturbances.
- D is the differential gain, which penalizes strong changes in the cross track error. Therefore it compensates "future" errors.

<iframe width="560" height="315" src="https://www.youtube.com/embed/4Y7zG48uHRo?rel=0" frameborder="0" allowfullscreen></iframe>


## PID tuning

To tune a PID controller for steering a vehicle in the center of the lane, the parameters can be tuned manually.
Starting with the P component, it should be increased until the vehicle stays in the center of the lane.
Furthermore the P gain should be kept small enought to avoid instable behavior, like overshooting.
Afterwards the D term can be increased slightly and should be kept low to avoid oscillating behavior of the steering.
Finally the I component can be increased until the cross track error is reduced further.

For fine tuning, the twiddle algorithm or stochastic gradient descent algorithm can be used.

Furthermore the throttle can be controlled by another PID controller which sets the speed to a desired velocity and slows down if the cross track error gets too large.

## Additional reading

- [PID Controller](https://en.wikipedia.org/wiki/PID_controller)
- [Controller](https://de.wikipedia.org/wiki/Regler)
- [Tuning](http://www.dee.ufrj.br/controle_automatico/artigos/ieee-edu2002.pdf)
