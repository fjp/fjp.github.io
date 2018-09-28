---
layout: single #collection
title: The Decorator Pattern
permalink: /design-patterns/decorator
excerpt: "The strategy design pattern summarized."
date: 2018-09-20 15:41:35 +0200
categories: [programming, design patterns]
tags: [programming, design patterns, decorator, pattern, structural]
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
<b>The Decorator Pattern</b> attaches additional
responsibilities to an object dynamically.
Decorators provide a flexible alternative to
subclassing for extending functionality
</p>
{: .notice}

The Decorator Pattern provides an alternative to subclassing for extending behavior. This is possible
with a set of decorator classes that are used to wrap concrete components. These
Decorator classes are of the same type as the components and therefore mirror the type of the components they decorate.
This is done either through inheritance or interface implementation. Decorators change the behavior of their components by adding
new functionality before and/or after (or even in place of) method calls to the component.

Using the decorator design pattern can result in many small objects, and overuse can be complex.

<figure>
    <a href="/assets/pages/design-patterns/decorator-pattern.png"><img src="/assets/pages/design-patterns/decorator-pattern.png"></a>
    <figcaption>Extensible and at the same time closed design with the decorator pattern.</figcaption>
</figure>

In the following example Beverage acts as a abstract component class.

{% highlight java %}
public abstract class Beverage {
	public enum Size { TALL, GRANDE, VENTI };
	Size size = Size.TALL;
	String description = "Unknown Beverage";

	public String getDescription() {
		return description;
	}

	public void setSize(Size size) {
		this.size = size;
	}

	public Size getSize() {
		return this.size;
	}

	public abstract double cost();
}
{% endhighlight %}

The concrete components implement this interface.

{% highlight java %}
public class Espresso extends Beverage {

	public Espresso() {
		description = "Espresso";
	}

	public double cost() {
		return 1.99;
	}
}
{% endhighlight %}

Another concrete component.

{% highlight java %}
public class DarkRoast extends Beverage {
	public DarkRoast() {
		description = "Dark Roast Coffee";
	}

	public double cost() {
		return .99;
	}
}
{% endhighlight %}


To extend the behavior of these concrete components the following condiment decorator inherits the `Beverage` base class
and has a member of type `Beverage`.

{% highlight java %}
public abstract class CondimentDecorator extends Beverage {
	public Beverage beverage;
	public abstract String getDescription();

	public Size getSize() {
		return beverage.getSize();
	}
}
{% endhighlight %}

Using these classes it is possible to wrap the concrete components which gives them
new behaviors. Note that this is not obtained through direct inheritance.
The next two code snippets show concrete decorators that implement the `CondimentDecorator` interface.

{% highlight java %}
public class Whip extends CondimentDecorator {
	public Whip(Beverage beverage) {
		this.beverage = beverage;
	}

	public String getDescription() {
		return beverage.getDescription() + ", Whip";
	}

	public double cost() {
		return beverage.cost() + .10;
	}
}
{% endhighlight %}

Another condiment decorator.

{% highlight java %}
public class Soy extends CondimentDecorator {
	public Soy(Beverage beverage) {
		this.beverage = beverage;
	}

	public String getDescription() {
		return beverage.getDescription() + ", Soy";
	}

	public double cost() {
		double cost = beverage.cost();
		if (beverage.getSize() == Size.TALL) {
			cost += .10;
		} else if (beverage.getSize() == Size.GRANDE) {
			cost += .15;
		} else if (beverage.getSize() == Size.VENTI) {
			cost += .20;
		}
		return cost;
	}
}
{% endhighlight %}

And another one.

{% highlight java %}
public class Mocha extends CondimentDecorator {
	public Mocha(Beverage beverage) {
		this.beverage = beverage;
	}

	public String getDescription() {
		return beverage.getDescription() + ", Mocha";
	}

	public double cost() {
		return beverage.cost() + .20;
	}
}
{% endhighlight %}

To instantiate a concrete component, or in this example, a beverage and decorate it with condiments,
the following main class can be executed.

{% highlight java %}
public class StarbuzzCoffee {

	public static void main(String args[]) {
		Beverage beverage = new Espresso();
		System.out.println(beverage.getDescription()
				+ " $" + String.format("%.2f", beverage.cost()));

		Beverage beverage2 = new DarkRoast();
		beverage2 = new Mocha(beverage2);
		beverage2 = new Mocha(beverage2);
		beverage2 = new Whip(beverage2);
		System.out.println(beverage2.getDescription()
				+ " $" + String.format("%.2f", beverage2.cost()));

		Beverage beverage3 = new HouseBlend();
		beverage3.setSize(Size.VENTI);
		beverage3 = new Soy(beverage3);
		beverage3 = new Mocha(beverage3);
		beverage3 = new Whip(beverage3);
		System.out.println(beverage3.getDescription()
				+ " $" + String.format("%.2f", beverage3.cost()));
	}
}
{% endhighlight %}

Starting this application results in the following output.

{% highlight bash %}
$ java StarbuzzCoffee
Espresso $1.99
Dark Roast Coffee, Mocha, Mocha, Whip $1.49
House Blend Coffee, Soy, Mocha, Whip $1.34
{% endhighlight %}
