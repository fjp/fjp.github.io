---
layout: single #collection
title: The Command Pattern
permalink: /design-patterns/command
excerpt: "The command design pattern summarized."
date: 2019-09-20 15:41:35 +0200
categories: [programming, design patterns]
tags: [programming, design patterns, command, pattern, behavioral]
comments: true
use_math: true
toc: false
# toc_label: "Unscented Kalman Filter"
classes: wide
header:
  #overlay_image: /assets/projects/autonomous-rc-car/hpi-racing-bmw-m3.png
  #overlay_filter: 0.5 # same as adding an opacity of 0.5 to a black background
  #caption: "Source: [**hpiracing**](http://www.hpiracing.com/de/kit/114343)"
  #show_overlay_excerpt: true
  #teaser: /assets/projects/autonomous-rc-car/hpi-racing-bmw-m3_thumb.png
  #overlay_image: /assets/projects/autonomous-rc-car/hpi-racing-bmw-m3.png
redirect_from:
    - /design-patterns/
sidebar:
    nav: "design-patterns"
author_profile: false
---

<p>
<b>The Command Pattern</b> encapsulates a request as an object, 
thereby letting you parametrize other objects with different requests, 
queue or log requests, and support undoable operations.
</p>
{: .notice}




<figure>
    <a href="/assets/pages/design-patterns/command-pattern.png"><img src="/assets/pages/design-patterns/command-pattern.png"></a>
    <figcaption>Encapsulate methods into Command objects with the command pattern: store them, pass them around, and invoke them when needed.</figcaption>
</figure>


The following example is a remote control with multiple slots, where each slot has a corresponding on/off button. 
Each button will have its own command object assigned. The command object know which actions to call on the receiver.

The `Command` interface has one `execute()` method:

{% highlight java %}
public interface Command {
	public void execute();
}
{% endhighlight %}

Concrete implementations of this interface for a light on/off command look like this:
For the on command:

{% highlight java %}
public class LightOnCommand implements Command {
	Light light;

	public LightOnCommand(Light light) {
		this.light = light;
	}

	public void execute() {
		light.on();
	}
}
{% endhighlight %}

And the off command:

{% highlight java %}
public class LightOffCommand implements Command {
	Light light;
 
	public LightOffCommand(Light light) {
		this.light = light;
	}
 
	public void execute() {
		light.off();
	}
}
{% endhighlight %}

To use these command objects an invoker, in this example the remote control, needs to be configured to hold these commands:

{% highlight java %}
public class RemoteControl {
	Command[] onCommands;
	Command[] offCommands;
 
	public RemoteControl() {
		onCommands = new Command[7];
		offCommands = new Command[7];
 
		Command noCommand = new NoCommand();
		for (int i = 0; i < 7; i++) {
			onCommands[i] = noCommand;
			offCommands[i] = noCommand;
		}
	}
  
	public void setCommand(int slot, Command onCommand, Command offCommand) {
		onCommands[slot] = onCommand;
		offCommands[slot] = offCommand;
	}
 
	public void onButtonWasPushed(int slot) {
		onCommands[slot].execute();
	}
 
	public void offButtonWasPushed(int slot) {
		offCommands[slot].execute();
	}
  
	public String toString() {
		StringBuffer stringBuff = new StringBuffer();
		stringBuff.append("\n------ Remote Control -------\n");
		for (int i = 0; i < onCommands.length; i++) {
			stringBuff.append("[slot " + i + "] " + onCommands[i].getClass().getName()
				+ "    " + offCommands[i].getClass().getName() + "\n");
		}
		return stringBuff.toString();
	}
}
{% endhighlight %}

Note that the constructor of this `RemoteControl` class assigns `NoCommand` objects to the slots.
The `NoCommand` object implements the `Command` interface but its `execute()` method does not do or return anything.

{% highlight java %}
public class NoCommand implements Command {
	public void execute() { }
}
{% endhighlight %}

<p>
Note that the <code>NoCommand</code> object is also sometimes considered a design pattern which is known as <b>Null</b>.
</p>
{: .notice}

the following main class can be executed.

{% highlight java %}

{% endhighlight %}

Starting this application results in the following output.

{% highlight bash %}

{% endhighlight %}
