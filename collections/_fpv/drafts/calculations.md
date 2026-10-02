---
layout: single
title:  "Thrust and Flight Time"
permalink: /projects/fpv/calculations/
excerpt: "Calculations to find matching parts and to estimate the flight time."
categories: [fpv, quad]
tags: [fpv, quad, race, drone, thrust, weight, flight time, calculation]
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

- Why calculating before buying parts helps

## Basic Calculations {#calculations}

- Electrical power $P = U \cdot I$ and energy of a battery

## Weight

- All-up weight from the [components](/projects/fpv/components) of this build

## Thrust

- Thrust-to-weight ratio, see [relation between thrust and weight](/projects/fpv/motor#relation-between-thrust-and-weight)

## Finding Matching Motors {#motors}

- Motor $K_v$, cell count and propeller size

## Finding Matching ESCs {#esc}

- Maximum motor current and ESC rating, see [ampere and load capacity](/projects/fpv/esc#ampere-a-and-load-capacity)

## Flight Time Calculation {#flight-time}

- Usable capacity $Q$ and average current: $t = \frac{0.8 \cdot Q}{I_{avg}}$
