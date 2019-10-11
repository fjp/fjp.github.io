---
layout: single #collection
title: The State Pattern
permalink: /design-patterns/state
excerpt: "The state design pattern summarized."
date: 2019-10-09 15:41:35 +0200
categories: [programming, design patterns]
tags: [programming, design patterns, state, pattern, behavioral]
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
<b>The State Pattern</b> allows an object to alter its behavior when its internal state changes.
The object will appear to change its class.
</p>
{: .notice}

The State Pattern allows an object to have many different behaviors that are based on its internal state. 
Unlike a procedural state machine, the State Pattern represents its states as individual classes, 
each inheriting from a common interface or abstract class.
This will typically result in a greater number of classes in your design. Beside this disadvantage,
using explicit classes for each state the code becomes easier to understand, maintain and it is more flexible.

The following class diagram shows an example, where the concrete state classes implement an abstract state interface.


<figure>
    <a href="/assets/pages/design-patterns/state-pattern.png"><img src="/assets/pages/design-patterns/state-pattern.png"></a>
    <figcaption>The State Pattern.</figcaption>
</figure>


The `Context` gets its behavior by delegating applied actions to the current state object it is composed with.
By encapsulating each state into a class, we localize any changes that will need to be made. 
This way we follow the principle: Encapsulate what varies (the state).
The principle: Open for extension but closed for modification is also coverd by the State Pattern.
Each state is closed for modification, and yet the `Context` is open for extension by adding new state classes.

The State and [Strategy Patterns](/design-patterns/strategy) have the same class diagram, but they differ in
intent. The Strategy Pattern typically configures Context classes with a behavior or algorithm.
State Pattern allows a Context to change its behavior as the state of the Context
changes.

State transitions can be controlled by the State classes or by the Context classes.

State classes may be shared among Context instances. 



If we require common methods, that are shared across states,
we use an abstract state class. Otherwise it is possible to use an interface. 
Using an abstract class has the benefit of allowing you to add methods to the abstract class later, 
without breaking the concrete state implementations.


## State Pattern Example

The following example shows a Gumball machine with the following states that will be represented by individual classes:

- NoQuarterState - The start state where the user hasn't inserted a quarter.
- HasQuarter - After inserting a quarter, we transition to this state.
- SoldState - This state is reached ff a the user inserted a quarter and truns the crank.
- SoldOut - If all gumballs are sold or the machine hasn't been filled, the machine transitions to this state

First, we’re going to define a `State` interface that contains a method for every action in the Gumball Machine.

{% highlight java %}
public interface State {
 
	public void insertQuarter();
	public void ejectQuarter();
	public void turnCrank();
	public void dispense();
	
	public void refill();
}
{% endhighlight %}

Now we are going to implement a `State` class for every state of the machine. 
These classes will be responsible for the behavior of the machine (`Context`) when it is in the corresponding state.
This way we delegate the work to the individual state classes.

The first state that will implement the `State` interface is `NoQuarterState`:

{% highlight java %}
public class NoQuarterState implements State {
    GumballMachine gumballMachine;
 
    public NoQuarterState(GumballMachine gumballMachine) {
        this.gumballMachine = gumballMachine;
    }
 
	public void insertQuarter() {
		System.out.println("You inserted a quarter");
		gumballMachine.setState(gumballMachine.getHasQuarterState());
	}
 
	public void ejectQuarter() {
		System.out.println("You haven't inserted a quarter");
	}
 
	public void turnCrank() {
		System.out.println("You turned, but there's no quarter");
	 }
 
	public void dispense() {
		System.out.println("You need to pay first");
	} 
	
	public void refill() { }
 
	public String toString() {
		return "waiting for quarter";
	}
}
{% endhighlight %}

Like all states will, this state has a reference to the `GumballMachine`, the `Context`, 
which is used to get and set new states. For example, to transition from this state `NoQuarterState` to `HasQuarterState` 
when the user inserts a coin, the `insertQuarter()` method uses the getter and setter of the `GumballMachine` class 
that is defined next:

{% highlight java %}
public class GumballMachine {
 
	State soldOutState;
	State noQuarterState;
	State hasQuarterState;
	State soldState;
 
	State state;
	int count = 0;
 
	public GumballMachine(int numberGumballs) {
		soldOutState = new SoldOutState(this);
		noQuarterState = new NoQuarterState(this);
		hasQuarterState = new HasQuarterState(this);
		soldState = new SoldState(this);

		this.count = numberGumballs;
 		if (numberGumballs > 0) {
			state = noQuarterState;
		} else {
			state = soldOutState;
		}
	}
 
	public void insertQuarter() {
		state.insertQuarter();
	}
 
	public void ejectQuarter() {
		state.ejectQuarter();
	}
 
	public void turnCrank() {
		state.turnCrank();
		state.dispense();
	}
 
	void releaseBall() {
		System.out.println("A gumball comes rolling out the slot...");
		if (count != 0) {
			count = count - 1;
		}
	}
 
	int getCount() {
		return count;
	}
 
	void refill(int count) {
		this.count += count;
		System.out.println("The gumball machine was just refilled; it's new count is: " + this.count);
		state.refill();
	}

	void setState(State state) {
		this.state = state;
	}
    public State getState() {
        return state;
    }

    public State getSoldOutState() {
        return soldOutState;
    }

    public State getNoQuarterState() {
        return noQuarterState;
    }

    public State getHasQuarterState() {
        return hasQuarterState;
    }

    public State getSoldState() {
        return soldState;
    }
 
	public String toString() {
		StringBuffer result = new StringBuffer();
		result.append("\nMighty Gumball, Inc.");
		result.append("\nJava-enabled Standing Gumball Model #2004");
		result.append("\nInventory: " + count + " gumball");
		if (count != 1) {
			result.append("s");
		}
		result.append("\n");
		result.append("Machine is " + state + "\n");
		return result.toString();
	}
}
{% endhighlight %}

The `GumballMachine` class instantiates all concrete states and provides all possible action methods to the user.
The action methods of this `Context` class delegate the work to the currently set state, which is stored in the `state` 
member. Note that `dispense()` requires no action method because it is an internal action of the Gumball machine. 
A user can't ask the machine to dispense directly. Instead, `dispense()` is called on the `state` object inside the
`turnCrank()` action method.

The rest of the states are implemented next. 

{% highlight java %}

{% endhighlight %}



{% highlight java %}

{% endhighlight %}



{% highlight java %}

{% endhighlight %}

result:

{% highlight bash %}
$ java MenuTestDrive

{% endhighlight %}
