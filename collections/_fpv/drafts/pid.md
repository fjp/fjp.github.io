---
layout: single
title:  "PID Values"
permalink: /projects/fpv/pid/
excerpt: "PID control and tuning of a FPV quad."
categories: [fpv, quad]
tags: [fpv, quad, race, drone, pid, tuning, betaflight]
comments: true
use_math: true
toc: true
classes: wide
sidebar:
  nav: "fpv"
# Draft: not built on GitHub Pages. Preview locally with `bundle exec jekyll serve --unpublished`.
# To publish remove this line, add a date and enable the matching entry in _data/navigation.yml.
published: false
---

## Introduction

- Why the [Flight Controller](/projects/fpv/flightcontroller) needs a controller for each axis (roll, pitch, yaw)

## PID Basics {#theory}

- Proportional, integral and derivative terms, see the [PID control](/control/pid/pid-control/) post
- Feedforward in Betaflight

## PID Tuning {#tuning}

- Betaflight PID tuning tab and filters
- Blackbox logs
- Tuning from the radio with the [PID Lua script](/projects/fpv/pid-lua-script)
