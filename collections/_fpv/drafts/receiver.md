---
layout: single
title:  "The RC Receiver"
permalink: /projects/fpv/receiver
excerpt: "The RC receiver receives the commands from the transmitter."
categories: [fpv, quad]
tags: [fpv, quad, race, drone, receiver, rx, sbus, fport]
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

Short introduction: the [receiver](/projects/fpv/glossar#receiver) receives the stick commands from the [transmitter](/projects/fpv/transmitter) and passes them to the [Flight Controller](/projects/fpv/flightcontroller).

## Protocols

- Receiver to Flight Controller: [SBUS](/projects/fpv/glossar#sbus), [SmartPort](/projects/fpv/glossar#smartport), [FPort](/projects/fpv/glossar#fport)
- Radio link: ACCST and [ACCESS](/projects/fpv/glossar#access), today also ExpressLRS and Crossfire

## Antennas

- Placement on the frame, see [receiver antennas](/projects/fpv/assembly#receiver-antennas)

## Failsafe

- What happens when the link is lost, see [failsafe mode](/projects/fpv/r-xsr#failsafe-mode)

## Receiver Used in this Build

- FrSky R-XSR, see [R-XSR page](/projects/fpv/r-xsr)
