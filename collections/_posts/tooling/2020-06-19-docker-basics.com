---
layout: single
title:  "Docker Basics"
date:   2020-06-19 19:31:41 +0200
excerpt: "Docker Container Basics."
permalink: /posts/tooling/docker/
categories: [docker, container, virtualization]
tags: [docker, container, tooling, virtualization, Dockerfile]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Kalman filter"
header:
  teaser: /assets/images/docker.png
  overlay_image: /assets/images/docker.png #keep it square 200x200 px is good
  overlay_filter: 0.5 # same as adding an opacity of 0.5 to a black background
  #caption: "Source: [**ROS Control Wiki**](http://wiki.ros.org/ros_control)"
  show_overlay_excerpt: true
---


Basic Dockerfile:

```Dockerfile
FROM alpine:latest


```


Basic Docker commands:

```
# run commands
docker container run
docker run
```
