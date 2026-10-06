---
layout: single
title:  "Rights and Regulations"
permalink: /projects/fpv/rights/
excerpt: "Laws, registration and insurance for flying FPV drones."
categories: [fpv, quad]
tags: [fpv, quad, race, drone, law, regulation, insurance]
date: 2026-10-06 09:00:00 +0200
comments: true
use_math: true
toc: true
classes: wide
sidebar:
  nav: "fpv"
---

## Introduction

Even a small FPV quad is an aircraft, so aviation rules apply to every flight.
They protect people on the ground and other airspace users, and they decide where and how you may fly.
In Austria the EU drone regulation applies, and Austro Control runs the registration and the online training on [dronespace.at](https://www.dronespace.at).

This page summarizes the rules as of October 2026 for a self-built FPV quad like the one of this project.
It is not legal advice: rules change, so check the current information on [dronespace.at](https://www.dronespace.at/faq) and from [EASA](https://www.easa.europa.eu/en/light/topics/drone-racing-and-flying-drone-goggles-first-person-view-fpv) before flying.
{: .notice--warning}

## Law and Insurance Regulations {#law-insurance}

### EU Drone Regulation and the Open Category

The EU regulations [2019/947](https://eur-lex.europa.eu/legal-content/EN/TXT/?uri=CELEX:32019R0947) (rules for operating drones)
and 2019/945 (requirements for drones and their class labels) apply in all EU member states, including Austria.
They divide drone flights into three categories by risk: open, specific and certified.
Hobby flights with a quad like this one belong to the open category, which needs no permission for each flight but limits where and how high you fly.
In the open category the maximum height is 120 m.

### Operator Registration

According to the [Austro Control FAQ](https://www.dronespace.at/faq), operators have to register if they fly drones from 250 g,
or drones of any weight with a sensor that can capture personal data, such as a camera.
An FPV quad always has a camera, so its operator has to register.

- The registration is done on [dronespace.at](https://www.dronespace.at), costs 46.80 EUR and is valid for 3 years.
- The operator receives a registration number, which has to be attached clearly visible to all drones of the operator.

### Online Training (A1/A3)

The pilot needs the competency certificate for the subcategories A1 and A3, the "Drohnenführerschein".
It consists of a free online course on dronespace.at and an online exam with 40 multiple choice questions. The minimum age is 16 years.

### Liability Insurance

Austro Control points out that the existing insurance obligations stay in force,
although the insurance documents are not checked during the registration.
Many household liability insurances exclude drones, so check the policy or take out a dedicated drone liability insurance before the first flight.

## Drones

### Weight Classes and Class Labels

New drones for the open category carry a class label from C0 to C4, which defines the subcategory they may be flown in:

| Class | Maximum take-off weight |
|:--|--:|
| C0 | below 250 g |
| C1 | below 900 g |
| C2 | below 4 kg |
| C3, C4 | below 25 kg |

Drones without a class label, which includes self-built quads, may be flown in subcategory A1 up to 249 g and in subcategory A3 above that (Austro Control FAQ).
The quad of this project weighs about 440 g with battery (see [Thrust and Flight Time](/projects/fpv/calculations/#weight)), so it is flown in subcategory A3.

In subcategory A3 you must not fly over people, you have to stay outside urban areas and at least 150 m away from residential, commercial or industrial areas, and the maximum height of 120 m applies ([EASA](https://www.easa.europa.eu/en/light/topics/drone-racing-and-flying-drone-goggles-first-person-view-fpv)).

### FPV Flights and the Observer

The open category requires that the drone stays in visual line of sight (VLOS).
With FPV goggles the pilot can't see the quad directly, so [EASA](https://www.easa.europa.eu/en/light/topics/drone-racing-and-flying-drone-goggles-first-person-view-fpv) requires a visual observer:

- The observer stands next to the pilot, not far away, and keeps the quad in sight.
- The observer must be able to talk to the pilot at any time.
- The observer must know the rules, but needs no qualification.
- Spectators are not allowed.

Without an observer, FPV flights in the open category are not allowed.

### Radio Equipment

The video transmitter and the RC link also have to follow the radio rules.
In the EU the 5.8 GHz video band may be used without a license with up to 25 mW e.i.r.p., see [output power](/projects/fpv/fpv-system/vtx#bands-channels-and-output-power).

## Checklist for this Build

1. Register as operator on dronespace.at and attach the registration number to the quad.
2. Pass the free A1/A3 online exam.
3. Make sure a liability insurance covers the quad.
4. Fly in subcategory A3: away from people and at least 150 m from residential, commercial and industrial areas, below 120 m.
5. Fly FPV only with an observer next to you.
6. Set the video transmitter to 25 mW on a channel inside the license-free band.
