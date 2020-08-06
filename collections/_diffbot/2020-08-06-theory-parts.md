---
layout: single #collection
title: Autonomous 2WD Robot - Theory of Components
permalink: /projects/diffbot/theory/parts/
excerpt: "Theory behind the components of the autonomous 2WD differential drive robot."
date: 2020-08-06 20:00:35 +0100
categories: [robotics]
tags: [2wd, differential drive, robot, ros, noetic, raspberry, pi, autonomous, theory, encoders, cameras]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Unscented Kalman Filter"
header:
  teaser: /assets/collections/diffbot/components/raspberry-pi-4.jpg
  overlay_image: /assets/collections/diffbot/components/raspberry-pi-4.jpg
  overlay_filter: 0.5 # same as adding an opacity of 0.5 to a black background
  # caption: "Source: [**hpiracing**](http://www.hpiracing.com/de/kit/114343)"
  show_overlay_excerpt: true
sidebar:
  nav: "diffbot"
---

In the following the theory behind some of the components of the 2WD differential drive robot are explained.

## Encoders

Encoders are attached to motors to report incremental movements which can be used to measure the position displacement and even speed or (noisy) acceleration. 
There are different types of encoders but for the differential drive robot we are focusing on rotational incremental encoders. 
Such encoders are used to measure the speed of a rotating motor or wheel.
There exist also quadrature encoders which can measure not only the wheel speed but also the direction the wheel is turning. 

References:

- [Wikipedia: Incremental encoder](https://en.wikipedia.org/wiki/Incremental_encoder)
- [DroneBotWorkshop](https://dronebotworkshop.com/robot-car-with-speed-sensors/), [YouTube Video](https://www.youtube.com/watch?v=oQQpAACa3ac)
- [What are pulses per revolution for encoders?](https://www.motioncontroltips.com/what-are-pulses-per-revolution-for-encoders/)
- [What is encoder PPR CPR and LPR](https://www.cuidevices.com/blog/what-is-encoder-ppr-cpr-and-lpr)
- [How encoder resolution is determined](https://www.linearmotiontips.com/how-encoder-resolution-is-determined/)
- [What is quadrature encoding](https://www.linearmotiontips.com/what-is-quadrature-encoding/)
- [Determining pulses per revolution for an encoder application](https://www.motioncontroltips.com/faq-how-do-i-determine-the-required-pulses-per-revolution-for-an-encoder-application/)
- [Comparing capacitive, optical, magnetic encoders](https://www.cuidevices.com/blog/capacitive-magnetic-and-optical-encoders-comparing-the-technologies)
- [LM393 Comperator Datasheet](https://datasheet.lcsc.com/szlcsc/3PEAK-LM393-SR_C93150.pdf)
- [Opto Interruper ITR8105 Datasheet](https://www.endrich.com/fm/2/ITR-8105.pdf)
- [Opto Interrupter Modules Dataseheet](https://www.eprolabs.com/wp-content/uploads/2016/07/SEN-0060.pdf)
- [Opto Interrupter Module](https://www.eprolabs.com/product/opto-interrupter-module/)
- [Adafruit T-Slot Photo Interrupter](https://www.adafruit.com/product/3986)

