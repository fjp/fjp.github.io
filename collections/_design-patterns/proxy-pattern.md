---
layout: single #collection
title: The Proxy Pattern
permalink: /design-patterns/proxy
excerpt: "The proxy design pattern summarized."
date: 2019-10-11 15:41:35 +0200
categories: [programming, design patterns]
tags: [programming, design patterns, proxy, pattern, structural]
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
<b>The Proxy Pattern</b> provides a surrogate or placeholder for another object to control access to it.
</p>
{: .notice}

The Proxy Pattern provides a representative for another object in order to control the client's access to it.
There are a number of ways it can manage that access.

- **Remote Proxy:** manages interaction between a client and a remote object.
- **Virtual Proxy:** controls access to an object that is expensive to instantiate.
- **Protection Proxy:** controls access to the methods of an object based on the caller.

Other variants of the Proxy Pattern are the following:

- **Firewall Proxy:** controls access to a set of network resources, protecting the subject from malicious clients. This is used in corporate firewall systems.
- **Smart Reference Proxy:** provides additional actions whenever a subject is referenced, such as counting the number of references to an object. In C++ this is a [shared smart pointer](https://en.cppreference.com/book/intro/smart_pointers#shared_ptr).
- **Caching Proxy:** provides t3emporary storage for results of operations that are expensive. It can also allow multiple clients to share the results to reduce computation or network latency. This is often seen in web server proxies as well as content management and publishing systems.
- **Synchronization Proxy:** provides safe access to a subject from multiple threas. This proxy is used in JavaSpaces, where it controls synchronized access to an underlying set of objects in a distributed environment. 
- **Complexity Hiding Proxy:** hides the complexity of and controls access to a complex set of classes. This is sometimes called the Facade Proxy. The Complexity Hiding Proxy differs from the [Facade Pattern](/design-patterns/facade) in that the proxy controls access, while the Facade Pattern just provides an alternative interface.
- **Copy-On-Write Proxy:** controls the opying of an object by deferring the copying of an object until it is required by a client. This is a variant of the Virtual Proxy.

<figure>
    <a href="/assets/pages/design-patterns/proxy-pattern.png"><img src="/assets/pages/design-patterns/proxy-pattern.png"></a>
    <figcaption>The Proxy Pattern.</figcaption>
</figure>


## Remote Proxy Pattern Example

The following example shows a company that has `GumballMachine`s which they want to monitor remotely.
To do this, we use a `GumballMonitor` that will talk to a proxy, which knows how to talk to the remote `GumballMachines` and how to return a report from the state of the `GumballMachine`.

The first step is to turn the `GumballMachine` into a service that can handle remote requests from clients. 
To do that, we need to:
1. Create a remote interface for the `GumballMachine`. This will provide a set of methods that can be called remotely.
2. Make sure all the return types in the interface are serializable.
3. Implement the interface in a concrete class.

Let's start with the remote interface:

{% highlight java %}
import java.rmi.*;
 
public interface GumballMachineRemote extends Remote {
	public int getCount() throws RemoteException;
	public String getLocation() throws RemoteException;
	public State getState() throws RemoteException;
}
{% endhighlight %}


The `State` class is not yet serializable. To fix this we use the following implementation:

{% highlight java %}
import java.io.*;
  
public interface State extends Serializable {
	public void insertQuarter();
	public void ejectQuarter();
	public void turnCrank();
	public void dispense();
}
{% endhighlight %}

Each `State` has a reference to a `GumballMachine` which should not be serialized.
In Java this can be achieved with the `transient` keyword as shown with the 
following `NoQuarterState` class.

{% highlight java %}
public class NoQuarterState implements State {
    private static final long serialVersionUID = 2L;
    transient GumballMachine gumballMachine;
 
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
 
	public String toString() {
		return "waiting for quarter";
	}
}
{% endhighlight %}

The `GumballMachine` class needs to act as a service and handle request coming over the network.
Therefore the `GumballMachine` needs to implement the `GumballMachineRemote` interface.

{% highlight java %}
import java.rmi.*;
import java.rmi.server.*;
 
public class GumballMachine
		extends UnicastRemoteObject implements GumballMachineRemote 
{
	private static final long serialVersionUID = 2L;
	State soldOutState;
	State noQuarterState;
	State hasQuarterState;
	State soldState;
	State winnerState;
 
	State state = soldOutState;
	int count = 0;
 	String location;

	public GumballMachine(String location, int numberGumballs) throws RemoteException {
		soldOutState = new SoldOutState(this);
		noQuarterState = new NoQuarterState(this);
		hasQuarterState = new HasQuarterState(this);
		soldState = new SoldState(this);
		winnerState = new WinnerState(this);

		this.count = numberGumballs;
 		if (numberGumballs > 0) {
			state = noQuarterState;
		} 
		this.location = location;
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

	void setState(State state) {
		this.state = state;
	}
 
	void releaseBall() {
		System.out.println("A gumball comes rolling out the slot...");
		if (count != 0) {
			count = count - 1;
		}
	}

	public void refill(int count) {
		this.count = count;
		state = noQuarterState;
	}
 
	public int getCount() {
		return count;
	}
 
    public State getState() {
        return state;
    }
 
    public String getLocation() {
        return location;
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

    public State getWinnerState() {
        return winnerState;
    }
 
	public String toString() {
		StringBuffer result = new StringBuffer();
		result.append("\nMighty Gumball, Inc.");
		result.append("\nJava-enabled Standing Gumball Model #2014");
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

To let the `GumballMachine` serve requests, we need to make sure we register it with the
RMI registry, which is Java's [Remote Method Invocation](https://en.wikipedia.org/wiki/Java_remote_method_invocation) registry. This allows clients to locate this service. The following test code takes care of the registration:

{% highlight java %}
import java.rmi.*;

public class GumballMachineTestDrive {
 
	public static void main(String[] args) {
		GumballMachineRemote gumballMachine = null;
		int count;

		if (args.length < 2) {
			System.out.println("GumballMachine <name> <inventory>");
 			System.exit(1);
		}

		try {
			count = Integer.parseInt(args[1]);

			gumballMachine = 
				new GumballMachine(args[0], count);
			Naming.rebind("//" + args[0] + "/gumballmachine", gumballMachine);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
}
{% endhighlight %}

The constructor of the `GumballMachine` can throw exceptions because it extends the Java `UnicastRemoteObject` class.
Therefore the creation of `GumballMachine` needs to be placed inside a `try` and `catch` statement.
The call to `Naming.rebind` publishes the `GumballMachine` stub under the name `gumballmachine`.

For the `GumballMonitor` to work over the network, we have to rely on the remote interface `GumballMachineRemote`.
This class will also catch any exceptions that might happen when requests over the network happen.

{% highlight java %}
import java.rmi.*;
 
public class GumballMonitor {
	GumballMachineRemote machine;
 
	public GumballMonitor(GumballMachineRemote machine) {
		this.machine = machine;
	}
 
	public void report() {
		try {
			System.out.println("Gumball Machine: " + machine.getLocation());
			System.out.println("Current inventory: " + machine.getCount() + " gumballs");
			System.out.println("Current state: " + machine.getState());
		} catch (RemoteException e) {
			e.printStackTrace();
		}
	}
}
{% endhighlight %}


To monitor a bunch of remote `GumballMachines` we can use the following test program:

{% highlight java %}
import java.rmi.*;
 
public class GumballMonitorTestDrive {
 
	public static void main(String[] args) {
		String[] location = {"rmi://santafe.mightygumball.com/gumballmachine",
		                     "rmi://boulder.mightygumball.com/gumballmachine",
		                     "rmi://seattle.mightygumball.com/gumballmachine"}; 
		
		if (args.length >= 0)
        {
            location = new String[1];
            location[0] = "rmi://" + args[0] + "/gumballmachine";
        }
		
		GumballMonitor[] monitor = new GumballMonitor[location.length];
		
		
		
		for (int i=0;i < location.length; i++) {
			try {
           		GumballMachineRemote machine = 
						(GumballMachineRemote) Naming.lookup(location[i]);
           		monitor[i] = new GumballMonitor(machine);
				System.out.println(monitor[i]);
        	} catch (Exception e) {
            	e.printStackTrace();
        	}
		}
 
		for(int i=0; i < monitor.length; i++) {
			monitor[i].report();
		}
	}
}
{% endhighlight %}

It uses the locations to monitor to create an array of `GumballMonitor`s.
Then it uses a `GumballMachineRemote` proxy to iterate through the state of each `GumballMachine`.


{% highlight bash %}
$ java TestDrive

{% endhighlight %}
