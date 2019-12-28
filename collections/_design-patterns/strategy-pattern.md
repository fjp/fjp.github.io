---
layout: single #collection
title: The Strategy Pattern
permalink: /design-patterns/strategy
excerpt: "The strategy design pattern summarized."
date: 2018-09-20 15:41:35 +0200
categories: [programming, design patterns]
tags: [programming, design patterns, strategy, pattern, behavioral]
comments: true
use_math: true
toc: false
# toc_label: "Unscented Kalman Filter"
classes: wide
header:
  teaser: /assets/pages/design-patterns/strategy-pattern.png
  overlay_image: /assets/pages/design-patterns/strategy-pattern.png
  #overlay_filter: 0.5 # same as adding an opacity of 0.5 to a black background
  #caption: "Source: [**hpiracing**](http://www.hpiracing.com/de/kit/114343)"
  #show_overlay_excerpt: true
redirect_from:
    - /design-patterns/
sidebar:
    nav: "design-patterns"
author_profile: false
---

<p>
<b>The Strategy Pattern</b> defines a family of algorithms,
encapsulates each one, and makes them interchangeable.
Strategy lets the algorithm vary independently from
clients that use it.
</p>
{: .notice}

<figure>
    <a href="/assets/pages/design-patterns/strategy-pattern.png"><img src="/assets/pages/design-patterns/strategy-pattern.png"></a>
    <figcaption>Encapsulated behavior with the strategy pattern.</figcaption>
</figure>


The Duck class that declares two reference variables for the behavior interface types.
All duck subclasses inherit these.

{% highlight java %}
public abstract class Duck {
	FlyBehavior flyBehavior;
	QuackBehavior quackBehavior;

	public Duck() {
	}

	public void setFlyBehavior(FlyBehavior fb) {
		flyBehavior = fb;
	}

	public void setQuackBehavior(QuackBehavior qb) {
		quackBehavior = qb;
	}

	abstract void display();

	public void performFly() {
		flyBehavior.fly();
	}

	public void performQuack() {
		quackBehavior.quack();
	}

	public void swim() {
		System.out.println("All ducks float, even decoys!");
	}
}
{% endhighlight %}


The interface that all flying behavior classes implement.

{% highlight java %}
public interface FlyBehavior {
	public void fly();
}
{% endhighlight %}

And some implementations of this interface:

{% highlight java %}
public class FlyWithWings implements FlyBehavior {
	public void fly() {
		System.out.println("I'm flying!!");
	}
}
{% endhighlight %}

{% highlight java %}
public class FlyNoWay implements FlyBehavior {
	public void fly() {
		System.out.println("I can't fly");
	}
}
{% endhighlight %}

{% highlight java %}
public class FlyRocketPowered implements FlyBehavior {
	public void fly() {
		System.out.println("I'm flying with a rocket");
	}
}
{% endhighlight %}

The quack behavior interface is similar to the fly behavior interface.

{% highlight java %}
public interface QuackBehavior {
	public void quack();
}
{% endhighlight %}

The following code snippets show two implementations of that interface:

{% highlight java %}
public class Quack implements QuackBehavior {
	public void quack() {
		System.out.println("Quack");
	}
}
{% endhighlight %}

{% highlight java %}
public class MuteQuack implements QuackBehavior {
	public void quack() {
		System.out.println("<< Silence >>");
	}
}
{% endhighlight %}

A subclass of `Duck` inherits the `quackBehavior` and `flyBehavior` instance variables.
If `performQuack()` is called on the `quackBehavior` member the responsibility for the quack
is delegated to the Quack behavior in the following example.

{% highlight java %}
public class MallardDuck extends Duck {

	public MallardDuck() {
		quackBehavior = new Quack();
		flyBehavior = new FlyWithWings();
	}

	public void display() {
		System.out.println("I'm a real Mallard duck");
	}
}
{% endhighlight %}


The following main application shows that with the strategy pattern it is
possible to change the behavior at runtime.

{% highlight java %}
public class MiniDuckSimulator {

	public static void main(String[] args) {

		Duck mallard = new MallardDuck();
		mallard.performQuack();
		mallard.performFly();

		Duck model = new ModelDuck();
		model.performFly();
		model.setFlyBehavior(new FlyRocketPowered());
		model.performFly();

	}
}
{% endhighlight %}


Starting this application results in the following output.

{% highlight bash %}
$java MiniDuckSimulator
Quack
I’m flying!!
I can’t fly
I’m flying with a rocket
{% endhighlight %}
