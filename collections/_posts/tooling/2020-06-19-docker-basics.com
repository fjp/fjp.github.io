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
  teaser: /assets/images/docker-logo.jpg
  overlay_image: /assets/images/docker-logo.jpg #keep it square 200x200 px is good
  overlay_filter: 0.5 # same as adding an opacity of 0.5 to a black background
  caption: "Source: [**t3n**](https://t3n.de/news/docker-hosting-613802/)"
  show_overlay_excerpt: true
---


Basic Dockerfile:

```bash
FROM alpine:latest


```


Basic Docker commands:

```console
# run commands
docker container run
docker run
```
