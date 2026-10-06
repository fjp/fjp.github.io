---
layout: single
title:  "LiPo Batteries"
permalink: /projects/fpv/battery
excerpt: "The LiPo battery stores the energy for a FPV quad."
categories: [fpv, quad]
tags: [fpv, quad, race, drone, battery, lipo]
date: 2026-10-06 09:00:00 +0200
comments: true
use_math: true
toc: true
classes: wide
header:
  teaser: /assets/collections/fpv/battery/battery-header.jpg
  overlay_image: /assets/collections/fpv/battery/battery-header.jpg
  overlay_filter: 0.5
sidebar:
  nav: "fpv"
---

[LiPo](/projects/fpv/glossar#lipo) (lithium polymer) batteries power almost all FPV quads.
They store a lot of energy for their weight and deliver the high currents that four motors draw at full throttle,
which is why race quads use them instead of NiMH packs.

<figure>
    <a href="/assets/collections/fpv/battery/turnigy-nano-tech-1300-3s.jpg"><img src="/assets/collections/fpv/battery/turnigy-nano-tech-1300-3s.jpg" alt="Turnigy nano-tech 1300 mAh 3S LiPo battery"></a>
    <figcaption>Turnigy nano-tech 1300 mAh 3S LiPo with the XT60 main lead and the JST-XH balance lead.</figcaption>
</figure>

## Cell Count and Voltage

A LiPo pack consists of cells connected in series. The number of cells is written with an S, so a 3S pack has three cells.
The voltage of a cell depends on its state of charge:

| State | Voltage per cell | 3S pack |
|:--|--:|--:|
| Fully charged | 4.2 V | 12.6 V |
| Nominal | 3.7 V | 11.1 V |
| Storage | 3.8 V | 11.4 V |
| Land (under load) | about 3.5 V | about 10.5 V |
| Damaged below | 3.0 V | 9.0 V |

The [glossar](/projects/fpv/glossar#battery) lists the nominal voltages of other cell counts.
The cell count determines how fast the motors spin, because a brushless motor turns at about $K_v$ times the voltage without load.
The EMAX RS2205 2300 $K_v$ [motors](/projects/fpv/motor) of this build reach about 25&nbsp;500 RPM on a 3S battery.
A battery with more cells needs motors with a lower $K_v$ for the same RPM.

## Capacity and C-Rating

The capacity $Q$ is given in milliampere hours (mAh). A 1300 mAh battery can deliver 1.3 A for one hour.
Together with the voltage it gives the stored energy, for a 3S 1300 mAh pack:

$$
E = U \cdot Q = 11.1\,\text{V} \cdot 1.3\,\text{Ah} \approx 14.4\,\text{Wh}
$$

A larger capacity allows longer flights, but it also makes the battery heavier.

The C-rating tells how much current a battery can deliver continuously, as a multiple of its capacity:

$$
I_{\max} = C \cdot Q
$$

The Turnigy nano-tech 1300 mAh pack is rated 45 to 90 C, which means $45 \cdot 1.3\,\text{A} = 58.5\,\text{A}$ continuously
and up to 117 A in short bursts. Manufacturers' C-ratings are often optimistic.
The internal resistance $R_i$ of the cells is a better measure: under load the voltage drops by $I \cdot R_i$,
so a current of 60 A causes a drop of 0.3 V per cell at an internal resistance of 5 mΩ. This voltage sag grows as a battery ages.

## Connectors

- **Main lead**: the XT60 plug carries the current to the [power distribution board](/projects/fpv/pdb). Its shape makes reversed polarity impossible.
- **Balance lead**: the white JST-XH plug has one pin more than the battery has cells, four pins on a 3S pack.
  The [charger](/projects/fpv/charger) uses it to balance the cells, and a battery checker uses it to measure each cell.

<figure>
    <a href="/assets/collections/fpv/battery/battery-xt60-pdb.jpg"><img src="/assets/collections/fpv/battery/battery-xt60-pdb.jpg" alt="LiPo battery connected to the power distribution board with an XT60 plug"></a>
    <figcaption>Battery connected to the XT60 lead of the PDB. The LEDs show that the 5 V and 10 V outputs are on.</figcaption>
</figure>

## Charging, Storage and Safety

- Charge LiPos only with a balance charger, by default at 1C, see [Charger and Testers](/projects/fpv/charger).
- If a battery won't be used for more than a few days, charge or discharge it to the storage voltage of 3.8 V per cell. Fully charged batteries age faster.
- Charge on a non-flammable surface or in a LiPo bag and never leave a charging battery unattended.
- Don't charge or fly batteries that are puffed or damaged, for example after a crash.
- Dispose of old batteries at a battery collection point, not in the household waste.

## Batteries of this Build

The [parts list](/projects/fpv/components) links a Lumenier 1300 mAh 3S 60C battery with an XT60 plug, which can deliver $60 \cdot 1.3\,\text{A} = 78\,\text{A}$ continuously.
The photos on this page show a Turnigy nano-tech 1300 mAh 3S pack with the same capacity, cell count and plug.
