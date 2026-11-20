---
layout: single
title: "From ETS to Home Assistant: Generating the KNX Configuration"
permalink: /projects/home-automation/ets-to-home-assistant/
excerpt: "How a KNX house with hundreds of group addresses gets into Home Assistant: the ETS project, generated YAML packages per room, areas and a phone dashboard, and the read-flag pitfall that makes status values wrong."
date: 2026-10-12 21:00:00 +0200
categories: [home automation, knx, home assistant]
tags: [home automation, knx, home assistant, ets, xknxproject, yaml, packages, generator, areas, dashboard, read flag, lesen, group monitor, troubleshooting]
comments: true
toc: true
sidebar:
  nav: "home-automation"
---

Our house runs on KNX: lights and dimmers, screens, heating, window and door contacts, sockets with power metering and a weather station.
Everything was planned in ETS, and a visualization server was the only user interface.
I wanted [Home Assistant](https://www.home-assistant.io/) as the main interface instead, mostly for its phone app and its automations.

That raised the first practical question: how do hundreds of group addresses get into Home Assistant without typing all of them by hand?
This page compares the options, and describes the route I took: a small generator that turns the ETS project into Home Assistant YAML, one file per room.
It ends with the pitfall that cost me the most time: two devices answering the same read.

## The options

| Option | Good for | Where the configuration lives |
|---|---|---|
| Home Assistant's KNX panel | A few dozen entities, created and edited in the UI | Home Assistant's own storage |
| YAML by hand | Small installations, or a few special entities | Your YAML files |
| [ets-to-homeassistant](https://github.com/laurent-martin/ets-to-homeassistant) | Converting a whole `.knxproj` once, structured by ETS functions | Generated YAML |
| [nickol-knx-mcp](https://github.com/NickoScope/nickol-knx-mcp) | Auditing and generating with an AI assistant | Generated YAML |
| An own generator | Many entities, house-specific rules, regenerating after every ETS change | Generated YAML in git |

**Home Assistant's KNX panel.** You import the ETS project into the panel. Home Assistant parses it with [xknxproject](https://github.com/XKNX/xknxproject) and then knows every group address with its name and data point type.
The group monitor shows telegrams with those names, and you can create entities in the UI and pick their group addresses from the project.
In Home Assistant 2026.9 the UI covers all 17 KNX platforms that YAML supports, from lights and covers to climate and sensors.
For a few dozen entities that is the easiest way, and it needs no YAML at all.

**ets-to-homeassistant** is a Ruby tool that converts a `.knxproj` into Home Assistant YAML.
It groups group addresses into devices using the ETS *functions* (or the building structure) and the data point types, and accepts a custom script for project-specific fixes.
If your project uses ETS functions consistently, it is a quick start.

**nickol-knx-mcp** is a newer Python project, also based on xknxproject: an MCP server for AI assistants that audits ETS projects (naming, data point types, command/status pairs) and can generate Home Assistant YAML.
It reads projects read-only and never talks to the bus. It describes itself as a public beta.

## Why I wrote a generator

Our project has hundreds of group addresses with very consistent names and numbering. Three things tipped the balance:

1. **The names carry the structure.** Each group address name starts with a room and device code, followed by the room, the device and the function, for example `GF2-L1 Office - Ceiling light - Switch`.
   The numbering is parallel: the middle group is the function (switch, status, brightness, …) and the sub-address is the device.
   A few rules turn that into entities.
2. **YAML in git can be reviewed.** Every change to the generator or to the ETS project shows up as a diff of the generated files before it reaches Home Assistant.
   After an ETS change I export the project again and regenerate everything; nothing is edited by hand.
3. **Lessons become rules.** Once I learned that a status address only helps when a real device answers it, that became one line in the generator instead of a note somewhere.

## The pipeline

```
ETS project (.knxproj)
  → imported in Home Assistant's KNX panel (parsed by xknxproject)
  → the generator reads the parsed project
  → one YAML package per room
  → copied to Home Assistant, configuration check, KNX reload
  → each entity assigned to its room (area) through the WebSocket API
  → the phone dashboard generated from the areas
```

The generator doesn't parse the `.knxproj` itself: it reads the project that Home Assistant has already parsed for its KNX panel.
That saves handling the project password and keeps both on the same parser version.
The catch: the parsed project sits in Home Assistant's internal storage, whose format is not a public interface. A tool for other people should read the `.knxproj` with xknxproject directly.

The output goes into Home Assistant [packages](https://www.home-assistant.io/docs/configuration/packages/): one file per room, each starting with "generated, do not edit".
Every entity gets a comment with the group addresses it uses, and anything the generator leaves out is printed with the reason, so nothing disappears silently.

## The rules that do the work

**Command and status belong together** when they share the device code and the main data point type: the light's switch address and its status, the screen's move and position addresses and its position status.

**A status address needs a device that answers.** If no device sends on it or answers reads, Home Assistant would read an address nobody serves. The generator only uses status addresses where a device has the transmit or the read flag.
The opposite problem, two devices answering the same address, is [the pitfall below](#pitfall-which-device-answers-the-read).

**Only real devices count.** An address linked only to the visualization server is not a function of the house. It is left out.

**Per platform:**
- Lights: switch and status, brightness, colour temperature and colour where the project has them.
- Covers: move, stop, position and position status, for screens and blinds.
- Climate: the room temperature controllers with their set points.
- Binary sensors: window and door contacts, with the polarity of the contact.
- Sensors: temperatures, brightness, wind, power readings.
- Energy: each individual power reading without an energy counter of its own gets an integrated kWh sensor for Home Assistant's Energy dashboard. Actuator totals are left out, so nothing is counted twice.

For four group addresses of a made-up office light:

| Group address | Name | Data point type |
|---|---|---|
| 20/0/1 | GF2-L1 Office - Ceiling light - Switch | 1.001 |
| 20/1/1 | GF2-L1 Office - Ceiling light - Status | 1.001 |
| 20/4/1 | GF2-L1 Office - Ceiling light - Brightness | 5.001 |
| 20/5/1 | GF2-L1 Office - Ceiling light - Brightness status | 5.001 |

the generated package contains:

```yaml
# Office: generated from the ETS project, do not edit; change the generator and rerun.
knx:
  light:
    # 20/0/1 GF2-L1 Office - Ceiling light - Switch / 20/1/1 … Status / 20/4/1 … Brightness / 20/5/1 … Brightness status
    - name: "Office Ceiling light"
      address: "20/0/1"
      state_address: "20/1/1"
      brightness_address: "20/4/1"
      brightness_state_address: "20/5/1"
```

The generator itself is still specific to our house: its room table, name patterns and a few overrides.
I may publish a generalized version later, if it turns out to be useful beyond our project.

## Rooms and the phone dashboard

Home Assistant groups entities by **area**. A second script puts every generated entity into the area of its room through the WebSocket API.
It only fills empty areas, so a room I assigned by hand stays as it is.

The phone dashboard is generated from the areas, too:

- **A start page that shows only what needs attention:** open windows and lights that are on, each list hidden while it is empty, plus a few energy values at the top.
- **Every room on the start page,** with its temperature and buttons for its lights and screens.
- **One page per floor,** with the controls of each room.
- **The weather and the outside lights on their own page.**

The script saves the dashboard through the API and writes a YAML copy into the repository, so a dashboard change is a reviewable diff as well.

<figure class="third">
  <a href="/assets/collections/home-automation/ets-to-home-assistant/dashboard-start.png"><img src="/assets/collections/home-automation/ets-to-home-assistant/dashboard-start.png" alt="Phone screenshot of the start page: energy values at the top (outside temperature, PV power and yield, consumption, heat pump, car), a card 'Offen' with a tilted window and an open roof window, and a card 'Licht an' with three lights."></a>
  <a href="/assets/collections/home-automation/ets-to-home-assistant/dashboard-rooms.png"><img src="/assets/collections/home-automation/ets-to-home-assistant/dashboard-rooms.png" alt="Phone screenshot of the room list on the start page: one card per room on two floors, each with its temperature and buttons for its lights and screens."></a>
  <a href="/assets/collections/home-automation/ets-to-home-assistant/dashboard-floor.png"><img src="/assets/collections/home-automation/ets-to-home-assistant/dashboard-floor.png" alt="Phone screenshot of a floor page: the living room with two lights and two screens, and the kitchen with two lights and a screen, each as a tile with a slider."></a>
  <figcaption>The generated dashboard on a phone, here for an invented demo house built with the same script (in German, like ours): the start page, the rooms, and a floor page.</figcaption>
</figure>

## Pitfall: which device answers the read?

Generated entities are only as good as the status values behind them.
After the switch, a few values were wrong in a strange way: the twilight value of the weather station stayed at 23 lx all night, although it was pitch dark outside, and other status values flipped back to an old value once an hour.
Nothing was broken on the bus. Two devices answered the same question, and Home Assistant believed whichever answer came last.
The addresses and devices in this section are a made-up example, not our installation.

### Write, read and response

KNX devices talk through group addresses with three kinds of telegrams:

- **GroupValueWrite**: a device sends a new value, for example a button switching a light on, or an actuator reporting its new state on a status address.
- **GroupValueRead**: a device asks "what is the current value of this address?". Visualizations and Home Assistant do this at start-up, and again later to stay in sync.
- **GroupValueResponse**: the answer to a read. Home Assistant and visualizations take it over as the new value; a KNX device does so where its object has the update flag (A, see below).

A read should get exactly **one** response: from the device that owns the value.
For a light's status, that is the actuator channel that switches the light. For a sensor value, it is the sensor.
Every other device linked to the address may listen, but it should not answer.

### The flags in ETS

Which device answers is decided by the flags of each communication object in ETS.
The German ETS shows them as letters:

| ETS (German) | ETS (English) | Meaning |
|---|---|---|
| K (Kommunikation) | C (Communication) | The object talks on the bus at all. |
| L (Lesen) | R (Read) | The object **answers read requests** with its value. |
| S (Schreiben) | W (Write) | The object accepts writes from the bus. |
| Ü (Übertragen) | T (Transmit) | The object sends its value by itself when it changes. |
| A (Aktualisieren) | U (Update) | The object takes over values from **responses**, not only from writes. |
| I (Lesen bei Init) | I (Read on Init) | The device sends a read for this object when it starts. |

The rule that prevents the problem: **per group address, only the object that owns the value has L (Lesen).**
Objects that only mirror a value (a visualization, a logic server, a second display) need K and usually A, but not L.

### An example with two answers

Take a small installation with three devices:

| Device | Individual address | Object on `22/0/1` "Twilight" | Flags |
|---|---|---|---|
| Weather station | 2.1.30 | Twilight value (owner) | K, L, Ü |
| Visualization server | 2.1.20 | Twilight value (mirror for its display) | K, **L**, S, A |
| Home Assistant | 2.1.250 | `sensor` entity for `22/0/1` | – |

The weather station sends the twilight value when it changes, in steps.
The visualization server keeps its own copy to show it, and because its object has L, it answers reads as well.

Its copy can go stale for many reasons. The server may restart and begin with an old or default value. It may have missed a telegram while it was busy, or its object may not take values over from responses.
The reason hardly matters: once the server answers reads, its stale copy goes on the bus as a proper response, and Home Assistant takes it over like any other answer.

At start-up, Home Assistant reads the state address of each KNX entity that has one, as long as state synchronization is on, which is the default. With the default `sync_state: true`, it also reads again when it heard nothing on an address for 60 minutes ([KNX integration docs](https://www.home-assistant.io/integrations/knx/)).
At night the twilight value doesn't change, so the station sends nothing, and Home Assistant reads once an hour. Each read gets two answers:

| Time | Source | Destination | Type | Value |
|---|---|---|---|---|
| 21:14:02.105 | 2.1.250 | 22/0/1 | GroupValueRead | |
| 21:14:02.131 | 2.1.30 | 22/0/1 | GroupValueResponse | 1 lx |
| 21:14:02.164 | 2.1.20 | 22/0/1 | GroupValueResponse | 23 lx |

Home Assistant keeps the last one, 23 lx. Which answer comes last is a race, so the value can flip between the two at every read.
In the history graph, that looks like a value that jumps at the same minute every hour, or that stays at a daytime value all night.

<figure>
  <a href="/assets/collections/home-automation/ets-to-home-assistant/two-responders.svg"><img src="/assets/collections/home-automation/ets-to-home-assistant/two-responders.svg" alt="Sequence diagram: Home Assistant sends a GroupValueRead on 22/0/1. The weather station answers 1 lx after 26 ms, the visualization server answers its stale 23 lx after 59 ms, and Home Assistant shows the last answer, 23 lx."></a>
  <figcaption>One read, two responses: the stale copy arrives last and wins.</figcaption>
</figure>

The same happens with any status: a light whose status reads "off" while it is on, or a switch for the shading automatic that shows "on" after someone switched it off.

### Finding the second responder on the bus

1. Open a group monitor: in ETS under *Diagnostics → Group Monitor*, or the group monitor in Home Assistant's KNX panel (opened from the KNX integration in the settings).
2. Send a read to the address in question: in the ETS group monitor with **Read** under *Group Functions*, or in Home Assistant under *Developer tools → Actions* with `knx.read` and the address.
3. Look at the responses. More than one **GroupValueResponse** from different source addresses means more than one device answers.

The source address tells you which device it is. If the values differ, you have found the reason for the wrong status.

### Finding candidates in the ETS project

A double responder is usually visible in the project before anything goes wrong: two communication objects with the communication and read flags on the same group address.
xknxproject reads the flags from a `.knxproj` export.
This lists every group address with more than one candidate responder, devices with an object that has both K and L:

```python
from collections import defaultdict

from xknxproject import XKNXProj

project = XKNXProj("house.knxproj", password=None).parse()

candidates = defaultdict(set)  # group address → devices whose object has K and L
for obj in project["communication_objects"].values():
    flags = obj["flags"]
    if flags["communication"] and flags["read"]:
        for address in obj["group_address_links"]:
            candidates[address].add(obj["device_address"])

for address, devices in sorted(candidates.items()):
    if len(devices) > 1:
        name = project["group_addresses"][address]["name"]
        print(f"{address:10} {name:40} {', '.join(sorted(devices))}")
```

For the example above it prints one line:

```
22/0/1     Twilight                                 2.1.20, 2.1.30
```

In our project the list was several hundred lines long, almost all of them with the visualization server next to the real device.
Treat the list as candidates to check in the group monitor, not as faults:

- A device can legitimately own a value that is also linked elsewhere.
- An object linked to several group addresses responds on its **sending address**, the one marked S in ETS. A read on one of its other addresses is answered on the sending address, not on the address you asked. The script lists such an object under every linked address.

### The fix

In ETS, for each address on the list:

1. Decide which object **owns** the value: the actuator channel, the sensor, or the logic that produces it.
2. On every other object linked to that address, **clear L (Lesen)**. Keep K. Keep S if the device should accept commands on it, and A if it should take over values from responses. Set Ü only where the device really produces the value.
3. Program the changed devices (the application is enough).
4. Repeat the read test: one response.

Before clearing flags on a visualization or logic server, check that it doesn't need them for its own logic outputs. Only the mirrors of values that another device owns lose L.

Home Assistant can become a responder too: a KNX `expose`, or an entity with `respond_to_read`, answers reads with Home Assistant's value.
The same rule applies: only where Home Assistant owns the value, and never next to a device that answers on the same address.

### On the Home Assistant side

**A KNX light shows "off" until its first status.**
Home Assistant's KNX light reports off while it has no state yet ([source, 2026.9.4](https://github.com/home-assistant/core/blob/2026.9.4/homeassistant/components/knx/light.py#L335): `is_on` returns `bool(state)`).
After a restart, a light shows off until its actuator answers the start-up read or sends a new state.
A light group or template built from those lights is therefore "all off" for a moment. Keep that in mind before an automation acts on it or sends it back to the bus.

**Fewer reads hide the problem, they don't fix it.**
`sync_state` can be set to `init` (read only at start-up) or `expire <minutes>`. With fewer reads you see the stale value less often, but the next read still gets two answers.
Fix the flag instead.

If you consume KNX telegrams in your own templates or automations (`knx_event`, or the `knx.telegram` trigger), each telegram carries its `source`.
Where you can't change the flags yet, accepting a value only from the owner's individual address is a robust stopgap.

### Checklist

- [ ] A value jumps at regular intervals, or shows a daytime value at night: suspect a second responder.
- [ ] Group monitor plus one read: how many responses, from which sources?
- [ ] List the group addresses with more than one K + L object in the project.
- [ ] Per address, keep L only on the owner; keep K, S, A and Ü where they belong.
- [ ] Program the devices and repeat the read test.
- [ ] Check that Home Assistant doesn't answer reads where a device owns the value.

## What I learned

- **Names are the real interface.** A consistent naming scheme in ETS made the generator possible. A typo in a group address name becomes a typo in the app, so I fix names in ETS and export again rather than patching them in Home Assistant.
- **Renames need care.** Home Assistant keeps an entity's ID when its name changes. When the generator renames an entity, I rename its ID on purpose, and check automations and the dashboard for the old one.
- **Read the status from the right device.** Generated entities are only as good as the status addresses behind them: see [the pitfall above](#pitfall-which-device-answers-the-read).
- **Review before deploy.** Every change goes through a pull request with an independent review and tests against a real Home Assistant version, before anything is copied to the house.

## Which option fits

- **Up to a few dozen entities, or you prefer clicking:** Home Assistant's KNX panel.
- **A whole project at once, with ETS functions:** try ets-to-homeassistant first.
- **You want an audit of the project as well:** have a look at nickol-knx-mcp.
- **Hundreds of entities, consistent names, and you want every change reviewed in git:** a small generator of your own on top of xknxproject.

## My setup

Home Assistant runs as a virtual machine on [Proxmox](https://www.proxmox.com/) on a Beelink EQ14 mini PC (Intel N150, 16 GB RAM), which I bought directly from [Beelink](https://www.bee-link.com/products/beelink-eq14-n150).
It talks to the bus through a KNX IP interface.

Tested with Home Assistant 2026.9 and its KNX integration. The ETS flags and the group monitor work the same in ETS 5 and ETS 6.
