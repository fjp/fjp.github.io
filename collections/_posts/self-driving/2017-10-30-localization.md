---
layout: single
title:  "Localization"
date:   2017-10-30 20:10:12 +0200
categories: [self-driving, localization, basics]
tags: [self-driving, udacity, localization]
use_math: true
toc: true
# toc_label: "Unscented Kalman Filter"
header:
  teaser: /assets/posts/2017-10-30-localization/localization.png
  overlay_image: /assets/posts/2017-10-30-localization/localization.png
  caption: "Source: [Udacity self driving car ND](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/2c318113-724b-4f9f-860c-cb334e6e4ad7/lessons/3b114d0b-36bd-4006-b48a-dcc6b1fb7d5d/concepts/0884e9b2-8b20-4b91-aa66-d7dbb665666d)"
---

Localization answers the following question:

Where is a car in a given map with a specified accuracy, about 10 cm or less.

GPS is commonly used to find a car with respect to the given map. However, GPS on its own
is not precise enough. Most of the time GPS has an accuracy of a width of a lane, about one to three meters.
Sometimes it can even be as broad as 10 to 50 meters.

To improve the localization reliability other onboard sensors of the self-driving vehicle are utilized in combination with a detailed map.
With the onboard sensor it is possible to measure distances to static obstacles, like trees poles or walls, which can be part of the map. These distances are measured
in the local coordinate system of the vehicle.

To estimate where the car is in the map it is necessary to match the observations with the map information.
This results in a transformation between both coordinates systems, the local car coordinate system and the coordinate system of the map.

The localization issue is solved if it is possible to estimate this transformation.

To sum up:

Onboard sensors are used to estimate a transformation between measurements and a given map.




{% include figure image_path="/assets/posts/2017-10-30-localization/localization.png" caption="Localizing a Self Driving Car (Source: [Udacity self driving car ND](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/2c318113-724b-4f9f-860c-cb334e6e4ad7/lessons/3b114d0b-36bd-4006-b48a-dcc6b1fb7d5d/concepts/0884e9b2-8b20-4b91-aa66-d7dbb665666d))" %}
