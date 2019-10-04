---
layout: single #collection
title: The Facade Pattern
permalink: /design-patterns/template-method
excerpt: "The facade design pattern summarized."
date: 2019-10-01 15:41:35 +0200
categories: [programming, design patterns]
tags: [programming, design patterns, template, method, pattern, behavioral]
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
<b>The Template Method Pattern</b> defines the skeleton of an algorithm in a method,
deferring some steps to subclasses. 
Template Method lets subclasses redefine certain steps of an algorithm without changing the algorithm's structure.
</p>
{: .notice}

The Template Method Pattern gives an important technique for code reuse. 
The template method's abstract class may define concrete methods, abstract methods, and hooks.
Abstract methods are implemented by subclasses and thereby alter the algorithm to their needs.
Hooks are methods that do nothing or default behavior in the abstract class, 
but may be overriden in the subclass to actually use it to manipulate the algorithm.
To prevent subclasses from chaning the algorithm in the template method, it is declared `final`.
Notice, that hooks make it possible to alter the behavior of the algorithm in the template method even if it is declared `final`.

<figure>
    <a href="/assets/pages/design-patterns/template-method-pattern.png"><img src="/assets/pages/design-patterns/template-method-pattern.png"></a>
    <figcaption>Template Method Pattern defines the steps of an algorithm and allows subclasses to provide the implementation for one or more steps.</figcaption>
</figure>

The Template Method Pattern follows the Hollywood Principled, which guides us to put decision making in high-level modules
that can decided how and when to call low-level modules: "don't call us, we call you".

The Template Method Pattern is not always designed by inheritance, as shown in the following example.
Instead many algorithms in Java and C++, for example `sort()` which implements the basic sorting algorithm for elements in a collection (List, Vector, HashMap, ...). For user defined types this method requires that the type implements the intferface `Comparable`, which declares a single method: `compareTo()`. This method is used by the algorithm in `sort()` to get the elements in the desired order.

The following example shows one possible implementation of the Template Method pattern.
It is about producing coffee and tea. The steps for each beverage are similar:

- Boil some water
- Brew
    - Coffee: Brew the coffee grinds
    - Tea: Steep the tea bag in the water
- Pour beverage in a cup
- Add condiment
    - Coffee: Add sugar and milk
    - Add lemon

The two recipes are essentialy the same except for brewing and adding condiments. 
The common steps are implemented in an abstract base class called `CaffeineBeverageWithHook` because both beverages 
contain caffeine. This class contains the template method `prepareRecipe()`, 
which serves as template for an algorithm, in this case, for making caffeinated beverages, and it is declared `final`.
Using this keyword, avoids overriding the actual algorithm in subclasses.
The intention of subclasses is to implement the `abstract` methods of the algorithm. 
In this example these are `brew()` and `addCondiments()`.
Additionaly subclasses can override the orther methods but don't have to,
because the default behavior is provided by the base class. The method `customerWantsCondiments()` is the hook,
that implements default behavior in the base class but is intended to be overriden in the subclasses.

{% highlight java %}
public abstract class CaffeineBeverageWithHook {
 
	final void prepareRecipe() {
		boilWater();
		brew();
		pourInCup();
		if (customerWantsCondiments()) {
			addCondiments();
		}
	}
 
	abstract void brew();
 
	abstract void addCondiments();
 
	void boilWater() {
		System.out.println("Boiling water");
	}
 
	void pourInCup() {
		System.out.println("Pouring into cup");
	}
 
	boolean customerWantsCondiments() {
		return true;
	}
}
{% endhighlight %}

The subclasses `CoffeeWithHook` and `TeaWithHook` are implementing the abstract `CaffeineBeverage` interface. 
Specifically, the abstract methods need to be implemented and the hook method is optional.
In this example, both subclasses implement the same hook method which could be moved to the base class.
Here, the hook methods ask the user for input on adding condiments:

{% highlight java %}
public class CoffeeWithHook extends CaffeineBeverageWithHook {
 
	public void brew() {
		System.out.println("Dripping Coffee through filter");
	}
 
	public void addCondiments() {
		System.out.println("Adding Sugar and Milk");
	}
 
	public boolean customerWantsCondiments() {

		String answer = getUserInput();

		if (answer.toLowerCase().startsWith("y")) {
			return true;
		} else {
			return false;
		}
	}
 
	private String getUserInput() {
		String answer = null;

		System.out.print("Would you like milk and sugar with your coffee (y/n)? ");

		BufferedReader in = new BufferedReader(new InputStreamReader(System.in));
		try {
			answer = in.readLine();
		} catch (IOException ioe) {
			System.err.println("IO error trying to read your answer");
		}
		if (answer == null) {
			return "no";
		}
		return answer;
	}
}
{% endhighlight %}

The `TeaWithHook` implements the `CaffeineBeverageWithHook` interface and implements the `brew()` and `addCondiments()` methods suitable for its receipe (algorithm). The hook method `customerWantsCondiments()` is also adapted to ask for the condiments that can be added to the tea beverage, a lemon:

{% highlight java %}
public class TeaWithHook extends CaffeineBeverageWithHook {
 
	public void brew() {
		System.out.println("Steeping the tea");
	}
 
	public void addCondiments() {
		System.out.println("Adding Lemon");
	}
 
	public boolean customerWantsCondiments() {

		String answer = getUserInput();

		if (answer.toLowerCase().startsWith("y")) {
			return true;
		} else {
			return false;
		}
	}
 
	private String getUserInput() {
		// get the user's response
		String answer = null;

		System.out.print("Would you like lemon with your tea (y/n)? ");

		BufferedReader in = new BufferedReader(new InputStreamReader(System.in));
		try {
			answer = in.readLine();
		} catch (IOException ioe) {
			System.err.println("IO error trying to read your answer");
		}
		if (answer == null) {
			return "no";
		}
		return answer;
	}
}
{% endhighlight %}

The 


{% highlight java %}
public class BeverageTestDrive {
	public static void main(String[] args) {
 
		TeaWithHook teaHook = new TeaWithHook();
		CoffeeWithHook coffeeHook = new CoffeeWithHook();
 
		System.out.println("\nMaking tea...");
		teaHook.prepareRecipe();
 
		System.out.println("\nMaking coffee...");
		coffeeHook.prepareRecipe();
	}
}
{% endhighlight %}


The output of the program is the following when we answer `y` for a lemon in our tea and `no` for not adding milk and sugar to our coffee:

{% highlight bash %}
$ java BeverageTestDrive
Making tea...
Boiling water
Steeping the tea
Pouring into cup
Would you like lemon with your tea (y/n)? y
Adding Lemon

Making coffee...
Boiling water
Dripping Coffee through filter
Pouring into cup
Would you like milk and sugar with your coffee (y/n)? n
{% endhighlight %}
