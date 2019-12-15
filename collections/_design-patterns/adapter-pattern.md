---
layout: single #collection
title: The Adapter Pattern
permalink: /design-patterns/adapter
excerpt: "The adapter design pattern summarized."
date: 2019-09-30 15:41:35 +0200
categories: [programming, design patterns]
tags: [programming, design patterns, adapter, pattern, structural]
comments: true
use_math: true
toc: false
# toc_label: "Unscented Kalman Filter"
classes: wide
header:
  teaser: /assets/pages/design-patterns/adapter-object-pattern.png
  overlay_image: /assets/pages/design-patterns/adapter-object-pattern.png
  #overlay_filter: 0.5 # same as adding an opacity of 0.5 to a black background
  #caption: "Source: [**hpiracing**](http://www.hpiracing.com/de/kit/114343)"
  #show_overlay_excerpt: true
  #overlay_image: /assets/projects/autonomous-rc-car/hpi-racing-bmw-m3.png
redirect_from:
    - /design-patterns/
sidebar:
    nav: "design-patterns"
author_profile: false
---

<p>
<b>The Adapter Pattern</b> converts the interface of a class into
another interface the clients expect. Adapter lets classes work 
together that couldn't otherwise because of incompatible interfaces.
</p>
{: .notice}

The Adapter Pattern is used when an existing adaptee class is needed and its interface is not the one a client needs.
For this, the pattern changes an intefcace into one a client expects using a adapter class. 
To achieve this, there are two forms of the Adapter Pattern: object and class adapters. 
Class adapters require multiple inheritance. They inherit from the adaptee and the target interface, 
which is expected by the client. With the object adapter, the adapter implements the target interface and wraps the adaptee by holding an instance of it.

The client is implemented against the target interface and uses the adapter in the following way:

1. The client makes a request to the adapter by calling a method on it using the target interface.
2. The adapter translates the request into one or more calls on the adaptee using the adaptee interface
3. The client receives the results of the call and never knows there is an adapter doing the translation.

<figure>
    <a href="/assets/pages/design-patterns/adapter-object-pattern.png"><img src="/assets/pages/design-patterns/adapter-object-pattern.png"></a>
    <figcaption>Object Adapter Pattern implements the Adaptee and uses composition for the target interface.</figcaption>
</figure>

Another way to implement an adapter in programming languages that support multiple inheritance like C++ is to let the adapter class inherit both the target interface and the adaptee. This is known as Class Adapter. An Object Adapter uses composition to pass requests to an Adaptee.

<figure>
    <a href="/assets/pages/design-patterns/adapter-class-pattern.png"><img src="/assets/pages/design-patterns/adapter-class-pattern.png"></a>
    <figcaption>Class Adapter Pattern subclasses the Target and the Adaptee.</figcaption>
</figure>


Adapters are similar to the [Facade Pattern](/design-patterns/facade) and the [Decorator Pattern](/design-patterns/decorator). An adapter wraps an object to change its interface, 
a decorator wraps an object to add new behaviors and responsibilities, 
and a facade "wraps" a set of objects to simplify.

The following example uses modified classes found in the example of the [Strategy Pattern](/design-patterns/strategy).
Here we have the following classes. A Duck interface acting as the target interface that a client expects to see: 

{% highlight java %}
public interface Duck {
	public void quack();
	public void fly();
}
{% endhighlight %}

An implementation of this target interface is the `MallardDuck`:

{% highlight java %}
public class MallardDuck implements Duck {
	public void quack() {
		System.out.println("Quack");
	}
 
	public void fly() {
		System.out.println("I'm flying");
	}
}
{% endhighlight %}

The object that we are going to adapt, the adaptee, is the following `Turkey` class:

{% highlight java %}
public interface Turkey {
	public void gobble();
	public void fly();
}
{% endhighlight %}


A concrete implementation of this interface is `WildTurkey`:

{% highlight java %}
public class WildTurkey implements Turkey {
	public void gobble() {
		System.out.println("Gobble gobble");
	}
 
	public void fly() {
		System.out.println("I'm flying a short distance");
	}
}
{% endhighlight %}

The followign main program uses adapters to adapt a `Turkey` to a `Duck`:

{% highlight java %}
public class DuckTestDrive {
	public static void main(String[] args) {
		MallardDuck duck = new MallardDuck();

		WildTurkey turkey = new WildTurkey();
		Duck turkeyAdapter = new TurkeyAdapter(turkey);

		System.out.println("The Turkey says...");
		turkey.gobble();
		turkey.fly();

		System.out.println("\nThe Duck says...");
		testDuck(duck);

		System.out.println("\nThe TurkeyAdapter says...");
		testDuck(turkeyAdapter);
		
		// Challenge
		Drone drone = new SuperDrone();
		Duck droneAdapter = new DroneAdapter(drone);
		testDuck(droneAdapter);
	}

	static void testDuck(Duck duck) {
		duck.quack();
		duck.fly();
	}
}
{% endhighlight %}

The adapter that is used here converts a `Turkey` to a `Duck`. Therefore it needs to implement the target interface `Duck`.
It has a reference to its adaptee, the `Turkey`, which is initialized through the constructor. The class implements the methods of its target interface using the adaptee reference:


{% highlight java %}
public class TurkeyAdapter implements Duck {
	Turkey turkey;
 
	public TurkeyAdapter(Turkey turkey) {
		this.turkey = turkey;
	}
    
	public void quack() {
		turkey.gobble();
	}
  
  // To fly the same distance as a duck, a turkey needs to fly five times as much
	public void fly() {
		for(int i=0; i < 5; i++) {
			turkey.fly();
		}
	}
}
{% endhighlight %}

The output of the program is:

{% highlight bash %}
$ java DuckTestDrive
The Turkey says...
Gobble gobble
I'm flying a short distance

The Duck says...
Quack
I'm flying

The TurkeyAdapter says...
Gobble gobble
I'm flying a short distance
I'm flying a short distance
I'm flying a short distance
I'm flying a short distance
I'm flying a short distance
{% endhighlight %}

Another real-life example is to use an adapter between Java iterators and enumerators.
In Java, early collection types (Vector, Stack, Hashtable, and a few others) implement a method, `elements()`, 
which returns an `Enumeration`. The `Enumeration` interface allows you to step through the elements of a collection 
without knowing the specifics of how they are managed in the collection.

{% highlight java %}
public interface Enumeration<E>
{
	public bool hasMoreElements(); // Tells if there are any more elements in the collection
	public E nextElement(); // Returns the next element in the collection/enumeration
}
{% endhighlight %}

In later versions of Java this was replaced with `Iterators` which has also a `remove()` method: 

{% highlight java %}
public interface Iterator<E>
{
	public bool hasNext(); // Tells if there are any more elements in the collection
	public E next(); // Returns the next element in the collection/iteration.
	public void remove(); // Removes from the underlying collection the last element returned by this iterator
}
{% endhighlight %}
	
To deal with legacy code, that exposes the `Enumeration` interface, yet we'd like for our new code to use only `Iterator`s.

An adapter to deal with this situation would need to implement the `Iterator` as its target interface and be composed with
`Enumeration` as its adaptee:

{% highlight java %}
public class EnumerationIterator implements Iterator<Object> {
	Enumeration<?> enumeration;
 
	public EnumerationIterator(Enumeration<?> enumeration) {
		this.enumeration = enumeration;
	}
 
	public boolean hasNext() {
		return enumeration.hasMoreElements();
	}
 
	public Object next() {
		return enumeration.nextElement();
	}
 
	public void remove() {
		throw new UnsupportedOperationException();
	}
}
{% endhighlight %}

The `hasNext()` and `next()` methods are straightforward to map from target to adaptee: we just pass them right through. The best we can do for `remove()` is to throw a runtime exception because `Enumeration` does not support removing.

The following code test the above `EnumerationIterator` adapter:

{% highlight java %}
public class EnumerationIteratorTestDrive {
	public static void main (String args[]) {
		Vector<String> v = new Vector<String>(Arrays.asList(args));
		// Pass old style Enumeration to the adapter
		Iterator<?> iterator = new EnumerationIterator(v.elements());
		// Now we can use the new style  Iterator methods
		while (iterator.hasNext()) {
			System.out.println(iterator.next());
		}
	}
}
{% endhighlight %}
