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
toc: true
toc_label: "Command Pattern"
classes: wide
header:
  teaser: /assets/pages/design-patterns/command-pattern.png
  overlay_image: /assets/pages/design-patterns/command-pattern.png
  #overlay_filter: 0.5 # same as adding an opacity of 0.5 to a black background
  caption: "Source: [**Head First Design Patterns**]({{ page.url }}/#reference)"
  #show_overlay_excerpt: true
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

- The Command Pattern decouples an object making a request, `invoker`, from the one that knows how to perform it, `receiver`.
- A `Command` object implements the `Command` interface and is at the center of this decoupling. A concrete `Command` object encapsulates a `receiver` with an action (or set of actions).
- An `invoker` makes a request of a `Command` object by calling its `execute()` method, which invokes those actions on the receiver.
- `invoker`s can be parameterized with `Commands`, even dynamically at runtime.
- `Commands` may support undo by implementing an `undo()` method that restores the object to its previous state before the `execute()` method was last called.
- `MacroCommands` are a simple extension of `Command` that allow multiple commands to be invoked. 
Likewise, `MacroCommand`s can easily support `undo()`.
- In practice, it is not uncommon for "smart" Command objects to implement the request themselves rather than delegating to a `receiver`.


Commands combine a receiver and its actions, which allows us to pass these packaged computations around and invoke them
at any time after creating the Command object. Clients that invoke such an object can be for example
scedulers, thread pools and job queus. 

In a job queue commands are queued and then dequeud by threads which call the `execute()` method of the `Command`.
After the call finished the command object is discarded and a new one is processed. 
This decouples the job queue classes from the objects that are doing teh computation. 
Such an application is useful for web servers that handle requests from multiple users.

`Command`s may also be used to implement logging and transactional systems. Executed commands are stored in a history on disk. When a crash occurs, the command objects are reloaded and invoked calling their `execute()` methods in batch and in order. To achieve this, the `Command` interface requires `store()` and `load()` methods. 
Such logging of command checkpoints is useful when working with large data, which would require time and space to store entire snapshots of the history, for example large text/spreadsheet documents or snapshots of virtual machines.


<figure>
    <a href="/assets/pages/design-patterns/command-pattern.png"><img src="/assets/pages/design-patterns/command-pattern.png"></a>
    <figcaption>Encapsulate methods into Command objects with the command pattern: store them, pass them around, and invoke them when needed.</figcaption>
</figure>


The following example is a remote control with multiple slots, where each slot has a corresponding on/off button. 
Each button will have its own command object assigned. The command object knows which actions to call on its receiver.
Additionally, there is an undo button on the remote which can also be implemented as `Command` as we will see later.

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

Here the `Light` class is a receiver which can look like this:

{% highlight java %}
public class Light {
	String location = "";

	public Light(String location) {
		this.location = location;
	}

	public void on() {
		System.out.println(location + " light is on");
	}

	public void off() {
		System.out.println(location + " light is off");
	}
}
{% endhighlight %}

A command with multiple actions looks like this:

{% highlight java %}
public class StereoOnWithCDCommand implements Command {
	Stereo stereo;
 
	public StereoOnWithCDCommand(Stereo stereo) {
		this.stereo = stereo;
	}
 
	public void execute() {
		stereo.on();
		stereo.setCD();
		stereo.setVolume(11);
	}
}
{% endhighlight %}

The off command of the Stereo is similar to the light off command:

{% highlight java %}
public class StereoOffCommand implements Command {
	Stereo stereo;
 
	public StereoOffCommand(Stereo stereo) {
		this.stereo = stereo;
	}
 
	public void execute() {
		stereo.off();
	}
}
{% endhighlight %}

To use these command objects an invoker, in this example the remote control, needs to be configured to hold these commands in its slots,
which is implemented with two arrays of `Command` type, one for `onCommands` and one for `offCommands`. The last button press is stored
in the `undoCommand` member:

{% highlight java %}
public class RemoteControlWithUndo {
	Command[] onCommands;
	Command[] offCommands;
	Command undoCommand;
 
	public RemoteControlWithUndo() {
		onCommands = new Command[7];
		offCommands = new Command[7];
 
		Command noCommand = new NoCommand();
		for(int i=0;i<7;i++) {
			onCommands[i] = noCommand;
			offCommands[i] = noCommand;
		}
		undoCommand = noCommand;
	}
  
	public void setCommand(int slot, Command onCommand, Command offCommand) {
		onCommands[slot] = onCommand;
		offCommands[slot] = offCommand;
	}
 
	public void onButtonWasPushed(int slot) {
		onCommands[slot].execute();
		undoCommand = onCommands[slot];
	}
 
	public void offButtonWasPushed(int slot) {
		offCommands[slot].execute();
		undoCommand = offCommands[slot];
	}
 
	public void undoButtonWasPushed() {
		undoCommand.undo();
	}
  
	public String toString() {
		StringBuffer stringBuff = new StringBuffer();
		stringBuff.append("\n------ Remote Control -------\n");
		for (int i = 0; i < onCommands.length; i++) {
			stringBuff.append("[slot " + i + "] " + onCommands[i].getClass().getName()
				+ "    " + offCommands[i].getClass().getName() + "\n");
		}
		stringBuff.append("[undo] " + undoCommand.getClass().getName() + "\n");
		return stringBuff.toString();
	}
}
{% endhighlight %}

Note that the constructor of this `RemoteControl` class assigns `NoCommand` objects to the slots.
The `NoCommand` object implements the `Command` interface but its `execute()` and `undo()` methods do nothing and do not return anything.

{% highlight java %}
public class NoCommand implements Command {
	public void execute() { }
	public void undo() { }
}
{% endhighlight %}

<p>
The <code>NoCommand</code> object is an example of a <b>null object</b>. 
A null object is useful when you don't have a meaningful object to return, 
and yet you want to remove the responsibility for handling null from the client.
</p>
{: .notice}

After having an option to assign commands to the invoker (remote), a client can set or programm the invoker:

{% highlight java %}
public class RemoteLoader {
 
	public static void main(String[] args) {
		RemoteControlWithUndo remoteControl = new RemoteControlWithUndo();
 
		Light livingRoomLight = new Light("Living Room");
		Light kitchenLight = new Light("Kitchen");
		CeilingFan ceilingFan= new CeilingFan("Living Room");
		GarageDoor garageDoor = new GarageDoor("");
		Stereo stereo = new Stereo("Living Room");
  
		LightOnCommand livingRoomLightOn = 
				new LightOnCommand(livingRoomLight);
		LightOffCommand livingRoomLightOff = 
				new LightOffCommand(livingRoomLight);
		LightOnCommand kitchenLightOn = 
				new LightOnCommand(kitchenLight);
		LightOffCommand kitchenLightOff = 
				new LightOffCommand(kitchenLight);
  
		CeilingFanOnCommand ceilingFanOn = 
				new CeilingFanOnCommand(ceilingFan);
		CeilingFanOffCommand ceilingFanOff = 
				new CeilingFanOffCommand(ceilingFan);
 
		GarageDoorUpCommand garageDoorUp =
				new GarageDoorUpCommand(garageDoor);
		GarageDoorDownCommand garageDoorDown =
				new GarageDoorDownCommand(garageDoor);
 
		StereoOnWithCDCommand stereoOnWithCD =
				new StereoOnWithCDCommand(stereo);
		StereoOffCommand  stereoOff =
				new StereoOffCommand(stereo);
 
		remoteControl.setCommand(0, livingRoomLightOn, livingRoomLightOff);
		remoteControl.setCommand(1, kitchenLightOn, kitchenLightOff);
		remoteControl.setCommand(2, ceilingFanOn, ceilingFanOff);
		remoteControl.setCommand(3, stereoOnWithCD, stereoOff);
  
		System.out.println(remoteControl);
 
		remoteControl.onButtonWasPushed(0);
		remoteControl.offButtonWasPushed(0);
		remoteControl.onButtonWasPushed(1);
		remoteControl.offButtonWasPushed(1);
		remoteControl.onButtonWasPushed(2);
		remoteControl.offButtonWasPushed(2);
		remoteControl.onButtonWasPushed(3);
		remoteControl.offButtonWasPushed(3);
	}
}
{% endhighlight %}

Starting this application results in the following output.

{% highlight bash %}
$java RemoteLoader
------ Remote control ------
[slot 0] LightOnCommand			LightOffCommand
[slot 1] LightOnCommand			LightOffCommand
[slot 2] CeilingFanOnCommand		CeilingFanOffCommand
[slot 3] StereoOnWithCDCommand		StereoOffCommand
[slot 4] NoCommand			NoCommand
[slot 5] NoCommand			NoCommand
[slot 6] NoCommand			NoCommand
[undo] NoCommand

Living Room light is on
Living Room light is off
Kitchen Room light is on
Kitchen Room light is off
Living Room ceiling fan is on
Living Room ceiling fan is off
Living Room sterio is on
Living Room sterio is set for CD input
Living Room sterio volume set to 11
Living Room light is off
{% endhighlight %}


Another example using the undo button is the following:

{% highlight java %}
public class RemoteLoader {
 
	public static void main(String[] args) {
		RemoteControlWithUndo remoteControl = new RemoteControlWithUndo();
 
		Light livingRoomLight = new Light("Living Room");
 
		LightOnCommand livingRoomLightOn = 
				new LightOnCommand(livingRoomLight);
		LightOffCommand livingRoomLightOff = 
				new LightOffCommand(livingRoomLight);
 
		remoteControl.setCommand(0, livingRoomLightOn, livingRoomLightOff);
 
		remoteControl.onButtonWasPushed(0);
		remoteControl.offButtonWasPushed(0);
		System.out.println(remoteControl);
		remoteControl.undoButtonWasPushed();
		remoteControl.offButtonWasPushed(0);
		remoteControl.onButtonWasPushed(0);
		System.out.println(remoteControl);
		remoteControl.undoButtonWasPushed();

		CeilingFan ceilingFan = new CeilingFan("Living Room");
   
		CeilingFanMediumCommand ceilingFanMedium = 
				new CeilingFanMediumCommand(ceilingFan);
		CeilingFanHighCommand ceilingFanHigh = 
				new CeilingFanHighCommand(ceilingFan);
		CeilingFanOffCommand ceilingFanOff = 
				new CeilingFanOffCommand(ceilingFan);
  
		remoteControl.setCommand(0, ceilingFanMedium, ceilingFanOff);
		remoteControl.setCommand(1, ceilingFanHigh, ceilingFanOff);
   
		remoteControl.onButtonWasPushed(0);
		remoteControl.offButtonWasPushed(0);
		System.out.println(remoteControl);
		remoteControl.undoButtonWasPushed();
  
		remoteControl.onButtonWasPushed(1);
		System.out.println(remoteControl);
		remoteControl.undoButtonWasPushed();
	}
}
{% endhighlight %}


And its output:

{% highlight bash %}
$java RemoteLoader
Light is on
Light is off

------ Remote control ------
[slot 0] LightOnCommand			LightOffCommand
[slot 1] NoCommand			NoCommand
[slot 2] NoCommand			NoCommand
[slot 3] NoCommand			NoCommand
[slot 4] NoCommand			NoCommand
[slot 5] NoCommand			NoCommand
[slot 6] NoCommand			NoCommand
[undo] LightOffCommand

Light is on

Ligt is off
Ligth is on

------ Remote control ------
[slot 0] LightOnCommand			LightOffCommand
[slot 1] NoCommand			NoCommand
[slot 2] NoCommand			NoCommand
[slot 3] NoCommand			NoCommand
[slot 4] NoCommand			NoCommand
[slot 5] NoCommand			NoCommand
[slot 6] NoCommand			NoCommand
[undo] LightOnCommand
{% endhighlight %}


## Macro Command

It is also possible to combine multiple commands into one:

{% highlight java %}
public class MacroCommand implements Command {
	Command[] commands;
 
	public MacroCommand(Command[] commands) {
		this.commands = commands;
	}
 
	public void execute() {
		for (int i = 0; i < commands.length; i++) {
			commands[i].execute();
		}
	}
	
     	//NOTE:  these commands have to be done backwards to ensure proper undo functionality
	public void undo() {
		for (int i = commands.length -1; i >= 0; i--) {
			commands[i].undo();
		}
	}
}
{% endhighlight %}

To create a macro the following steps need to be done:

1. Create the set of commands for the macro
2. Create two arrays, one for the On commands and one for the Off commands
3. Create new `MacroCommand` objects for the On and Off macros
4. Asign the `MacroCommand`s to a button using the `RemoteControl.setCommand()` method.
5. Pushing a button that has a `MacroCommand` assigned will invoke the specified actions. 

For the undo functionality of a `MacroCommand` all the commands that were invoked in the macro must undo their previous actions.
As shown in the previous code snippet, the commands need to be done backwards, to ensure proper undo functionality.


To implement a history of undo commands, in order to press the undo button multiple times, a stack of previous commands instead of just a reference to the last command is need. Then, whenever undo is pressed, 
the invoker pops the first item (command) off the stack and calls its `undo()` method.


## Lambda Expressions Implementation

To avoid having multiple small command classes that only have one method (`execute()`), 
which provide a common interface to the behavior of many different receivers, we can use lambda expressions/functions instead.

<p>
Note that a lambda expression can only be used if its arguments and return type matches exactly one and only one method
in an interface. Lambda expressions are designed specifically to replace the methods inthese functional interfaces,
partly as a way to reduce the code that is required when you have a lot of these small classes with functional interfaces.
If the interface has two methods, it's not a functional interfae and it won't be possible to replace it with lambda expressions.
</p>
{: .notice}

To achieve this we create a lambda expression, also called anonymous function, that is a function (or a subroutine) defined, and possibly called, without being bound to an identifier. In the example above, the lambda expression should call the 
`execute()` method. To use lambda expressions the following steps are required:

1. Create the Receiver, which is the same as before

{% highlight java %}
Light livingRoomLight = new Light("Living Room");
{% endhighlight %}

2. Set the remote control's commands using lambda expressions

Instead of creating `LightOnCommand` and `LightOffCommand` objects to pass to the `remoteControl.setCommand()`, we simply pass a lambda expression in place of each object, with the code from their respective `execute()` method:

{% highlight java %}
remoteControl.setCommand(0, () -> { livingRooomLight.on() }, () -> { livingRoomLight.off() }; } );

public void setCommand(int slot, Command onCommand, Command offCommand) {
	onCommands[slot] = onCommand;
	offCommands[slot] = offCommand;
}
{% endhighlight %}

3. Push the remote control buttons

When we call the remote's `onButtonWasPushed(0)` method, the command that's in slot 0 is a function object (created by the lambda expression). 
Because the lambda expression has the same signature as the `execute()` method of the `Command` interface, the compiler is able to match this method with the lambda expression. 
Both have no arguments and no return types.
Therefore, calling `onButtonWasPushed(0)` invokes `onCommands[0].execute()` which the lambda expressions stands in for and its statements are executed.

Instead of using lambda expressions which call only one method, for example `livingRoomLight.on();`, it is possible to simplify the code using *method references*.

{% highlight java %}
remoteControl.setCommand(0, livingRoomLight.on, livingRoomLight.off);
{% endhighlight %}

In case we need to call more than one method we need to create a lambda expression either in line,
or we can cwrite it separately, give it a name, and then pass this to the `remoteControl`'s `setCommand()` method. 
For example with the `sereoOnWithCDCommand` that does three things:

{% highlight java %}
stereo.on();
stereo.setCD();
stereo.setVolume(11);
{% endhighlight %}

a named lambda expressions, that has type `Command` to match the `Command` interface's `execute()` method, would look like this:

{% highlight java %}
Command stereoOnWithCD = () -> {
	stereo.on();
	stereo.setCD();
	stereo.setVolume(11);
}
{% endhighlight %}

This can then be passed using its name:


{% highlight java %}
remoteControl.setCommand(3, stereoOnWithCD, stereo::off);
{% endhighlight %}

Testing the remote control with lambda expressions with the following main program:

{% highlight java %}
public class RemoteControlTest {
	public static void main(String[] args) {
		RemoteControl remote = new RemoteControl();
		
		Light light = new Light();
		GarageDoor garageDoor = new GarageDoor();
		Stereo steero = new Stereo("Living Room");
		
		Command stereoOnWithCD = () -> {
			stereo.on();
			stereo.setCD();
			stereo.setVolume(11);
		}
		
		remote.setCommand(0, light::on, light::off);
		remote.setCommand(1, garageDoor::up, garageDoor::down);
		remote.setCommand(2, stereoOnWithCD, stereo::off);
		
		
		remote.onButtonWasPushed(0);
		remote.offButtonWasPushed(0);
		remote.onButtonWasPushed(1);
		remote.offButtonWasPushed(1);
		remote.onButtonWasPushed(2);
		remote.offButtonWasPushed(2);
    }

}
{% endhighlight %}


{% highlight bash %}
$java RemoteLoader

Living Room light is on
Living Room light is off
Garage Door is up
Garage Door is down
Living Room stereo iso n
Living Room stero is set for CD input
Living Room stereo volume set to 11
Living Room stereo is off
{% endhighlight %}

The constructor of the `RemoteControl` can also be adapted to use lambda expressions instead of `NoCommand` objects to reduce the number of classes required.

{% highlight java %}
public class RemoteControl {
	Command[] onCommands;
	Command[] offCommands;
	
	public RemoteControl() {
		onCommands = new Command[7];
		offCommands = new Command[7];
		
		for (int i = 0; i < 7; i++) {
			onCommands[i] = () -> { };
			offCommands[i] = () -> { };
		}
	}
	// rest of the code here
}
{% endhighlight %}
