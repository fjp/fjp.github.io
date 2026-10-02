---
layout: single
title:  "ESC Settings"
permalink: /projects/fpv/esc-settings
excerpt: "Configuring the ESCs of the race quad."
categories: [fpv, quad]
tags: [fpv, quad, race, drone, esc, blheli, dshot]
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

Short introduction: which [ESC](/projects/fpv/esc) settings matter and where they are configured.

## Configuration Tool

- BLHeliSuite32 via the Betaflight passthrough
- BLHeli_32 is no longer developed, newer ESCs use AM32 or Bluejay

## Motor Protocol

- DShot in Betaflight

## Motor Direction

- Reversing motor direction in software instead of swapping wires

## ESC Telemetry

- Current and RPM data for the Flight Controller
