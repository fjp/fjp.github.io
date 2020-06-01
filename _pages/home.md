---
title: "fjp.github.io"
layout: splash
permalink: /
date: 2018-04-22T11:48:41-04:00
header:
  overlay_color: "#000"
  overlay_filter: "0.5"
  overlay_image: /assets/images/unsplash-image-1.jpg
  cta_label: "github.com/fjp"
  cta_url: "https://github.com/fjp/"
  caption: "Photo credit: [**Unsplash**](https://unsplash.com)"
excerpt: "The wheel was man’s greatest invention – until he got behind it. (Bill Ireland)"
intro:
  - excerpt: "Explore the technology of self-driving vehicles, robotics, linux, ... and more."
feature_row:
  - image_path: /assets/posts/2017-12-03-trajectory-following/trajectory-following-overview.png
    alt: "Trajectory Following"
    title: "Trajectory Following"
    excerpt: "Where should your robot move in the next few seconds? This important question and even more can be solved by **following trajectories**"
  - image_path: /assets/posts/2017-10-30-localization/localization.png
    alt: "Localization"
    title: "Localization"
    excerpt: "Localization is the key for a robot to navigate in a world. Learn here some localization techniques."
    url: "/posts/optimal-frenet/"
    btn_label: "Read More"
    btn_class: "btn--primary"
  - image_path: /assets/posts/2017-12-07-model-predictive-control/timesteps-horizonlength.png
    title: "Model Predictive Control"
    excerpt: "Optimal control is achieved with **Model Predictive Control**. Find out how it works and learn about its strengths and weaknesses."
feature_row2:
  - image_path: /assets/posts/2017-12-07-model-predictive-control/mpc-constraints.png
    alt: "Path Planning"
    title: "Path Planning"
    excerpt: 'What is your overall goal? Not only you should ask yourself this question sometimes. A robot requires an answer to this important question too, in order to know where it is going. To help him, read about **Path Planning** first.'
    url: "/posts/path-planning/"
    btn_label: "Read More"
    btn_class: "btn--primary"
feature_row3:
  - image_path: /assets/posts/2017-10-17-ctrv-model/ctrv-model.png
    alt: "CTRV Model"
    title: "CTRV Model"
    excerpt: 'The Constant Turn Rate and Constant Velocity model. One of the kinematic vehicle models.'
    url: "/posts/vehicle-models/"
    btn_label: "Read More"
    btn_class: "btn--primary"
feature_row4:
  - image_path: /assets/posts/2017-10-15-unscented-kalman-filter/ukf-prediction.png
    alt: "ukf prediction"
    title: "UKF Prediction"
    excerpt: 'The famous **Unscented Kalman Filter** explained.'
    url: "/blog/ukf"
    btn_label: "Read More"
    btn_class: "btn--primary"
---

{% include feature_row id="intro" type="center" %}

{% include feature_row %}

{% include feature_row id="feature_row2" type="left" %}

{% include feature_row id="feature_row3" type="right" %}

{% include feature_row id="feature_row4" type="center" %}
