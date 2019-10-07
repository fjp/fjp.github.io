---
layout: single #collection
title: The Iterator Pattern
permalink: /design-patterns/iterator
excerpt: "The iterator design pattern summarized."
date: 2019-10-03 15:41:35 +0200
categories: [programming, design patterns]
tags: [programming, design patterns, iterator, pattern, behavioral]
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
<b>The Iterator Pattern</b> provides a way to 
access the elements of an aggreagate object sequentially 
without exposing its underlying respresentation.
</p>
{: .notice}

An Iterator allows access to an aggregate's (a collection) elements without exposing its internal structure.
Iterating over an aggreagate using an iterator encapsulates this task in another object other than the aggregate itself.
Thereby we relieve the aggregate of the responsibility of supporting operations for traversing its data.
The iterator provides a common interface for traversing the items of an aggregate, allowing you to use polymporphism when writing code that makes use of the items of the aggregate. In other words, when we write methods that take iterators as parameters, we are using polymorphic iteration. That means we asre creating cdode that can iterate over any collection as long as it supports the `Iterator` interface. The implementation of the underlying collection doesn't matter, we can still write code to iterate over it.

The Iterator Pattern is commonly used with the [Composite Pattern](/design-patterns/composite) to iterate over its components.


<figure>
    <a href="/assets/pages/design-patterns/iterator-pattern.png"><img src="/assets/pages/design-patterns/iterator-pattern.png"></a>
    <figcaption>Iterator Pattern uses a Factory Method `createIterator()` to create an iterator for an Aggregate.</figcaption>
</figure>

We distinguish between "internal" and "external" iterators.
Using an external iterator, the client controls the iteration by calling `next()` to get the next element. An internal iterator is controlled by the iterator iself. This way, the iterator is stepping through the elements and thereby controls 
the iteration itself. To get apply operations on the elements we have to pass the internal iterator a method. With internal iterators the client doesn't have control of the iteration, which might not be required if a single operation should be applied to all elements.


The following example shows two restraunt menus, where both implement the same aggregate interface `Menu`. Each menu has menu items stored in different types of collections. With the Iterator Pattern it is possible to iterate over these items without 
knowing the underlying type of the aggregate.

Each aggregate implements the `createIterator()` method which is declared in the `Menu` interface:

{% highlight java %}
import java.util.Iterator;

public interface Menu {
	public Iterator<?> createIterator();
	
	String name;
	public String getName() {
		return name;
	}
}
{% endhighlight %}

Each menu will have a menu item that implements the following interface:

{% highlight java %}
public class MenuItem {
	String name;
	String description;
	boolean vegetarian;
	double price;
 
	public MenuItem(String name, 
	                String description, 
	                boolean vegetarian, 
	                double price) 
	{
		this.name = name;
		this.description = description;
		this.vegetarian = vegetarian;
		this.price = price;
	}
  
	public String getName() {
		return name;
	}
  
	public String getDescription() {
		return description;
	}
  
	public double getPrice() {
		return price;
	}
  
	public boolean isVegetarian() {
		return vegetarian;
	}
}
{% endhighlight %}


Next we define the two menu classes (aggregates). `PancakeHouseMenu` uses an `ArrayList<MenuItem>` for its items.

{% highlight java %}
public class PancakeHouseMenu implements Menu {
	ArrayList<MenuItem> menuItems;
 
	public PancakeHouseMenu() {
		name = "BREAKFAST";
		menuItems = new ArrayList<MenuItem>();
    
		addItem("K&B's Pancake Breakfast", 
			"Pancakes with scrambled eggs, and toast", 
			true,
			2.99);
 
		addItem("Regular Pancake Breakfast", 
			"Pancakes with fried eggs, sausage", 
			false,
			2.99);
 
		addItem("Blueberry Pancakes",
			"Pancakes made with fresh blueberries, and blueberry syrup",
			true,
			3.49);
 
		addItem("Waffles",
			"Waffles, with your choice of blueberries or strawberries",
			true,
			3.59);
	}

	public void addItem(String name, String description,
	                    boolean vegetarian, double price)
	{
		MenuItem menuItem = new MenuItem(name, description, vegetarian, price);
		menuItems.add(menuItem);
	}
 
	public ArrayList<MenuItem> getMenuItems() {
		return menuItems;
	}
  
	public Iterator<MenuItem> createIterator() {
		return menuItems.iterator();
	}
  
	// other menu methods here
}
{% endhighlight %}

The member `menuItems` is an `ArrayList` which implements the `Iterator` interface and therefore provides the `iterator` method that returns an iterator to the elements of the `ArrayList`. 

The next concrete aggregate that implements the `Menu` interface is the `DinerMenu` class. Because it uses a standard array we will need a `DinerMenuIterator` defined afterwards:

{% highlight java %}
import java.util.Iterator;

public class DinerMenu implements Menu {
	static final int MAX_ITEMS = 6;
	int numberOfItems = 0;
	MenuItem[] menuItems;
  
	public DinerMenu() {
		name = "LUNCH";
		menuItems = new MenuItem[MAX_ITEMS];
 
		addItem("Vegetarian BLT",
			"(Fakin') Bacon with lettuce & tomato on whole wheat", true, 2.99);
		addItem("BLT",
			"Bacon with lettuce & tomato on whole wheat", false, 2.99);
		addItem("Soup of the day",
			"Soup of the day, with a side of potato salad", false, 3.29);
		addItem("Hotdog",
			"A hot dog, with saurkraut, relish, onions, topped with cheese",
			false, 3.05);
		addItem("Steamed Veggies and Brown Rice",
			"Steamed vegetables over brown rice", true, 3.99);
		addItem("Pasta",
			"Spaghetti with Marinara Sauce, and a slice of sourdough bread",
			true, 3.89);
	}
  
	public void addItem(String name, String description, 
	                     boolean vegetarian, double price) 
	{
		MenuItem menuItem = new MenuItem(name, description, vegetarian, price);
		if (numberOfItems >= MAX_ITEMS) {
			System.err.println("Sorry, menu is full!  Can't add item to menu");
		} else {
			menuItems[numberOfItems] = menuItem;
			numberOfItems = numberOfItems + 1;
		}
	}
 
	public MenuItem[] getMenuItems() {
		return menuItems;
	}
  
	public Iterator<MenuItem> createIterator() {
		return new DinerMenuIterator(menuItems);
		//return new AlternatingDinerMenuIterator(menuItems);
	}
 
 	public 
 
	// other menu methods here
}
{% endhighlight %}

Also `DinerMenu` returns its concrete implementation of the `Iterator<MenuItem>` interface, `DinerMenuIterator`:

{% highlight java %}
import java.util.Iterator;
  
public class DinerMenuIterator implements Iterator<MenuItem> {
	MenuItem[] list;
	int position = 0;
 
	public DinerMenuIterator(MenuItem[] list) {
		this.list = list;
	}
 
	public MenuItem next() {
		MenuItem menuItem = list[position];
		position = position + 1;
		return menuItem;
	}
 
	public boolean hasNext() {
		if (position >= list.length || list[position] == null) {
			return false;
		} else {
			return true;
		}
	}
 
	public void remove() {
		if (position <= 0) {
			throw new IllegalStateException
				("You can't remove an item until you've done at least one next()");
		}
		if (list[position-1] != null) {
			for (int i = position-1; i < (list.length-1); i++) {
				list[i] = list[i+1];
			}
			list[list.length-1] = null;
		}
	}
}
{% endhighlight %}



The client in this example is the `Waitress` which stores the `menus` in an `ArrayList<Menu>` and uses iterator from `java.util`. Using the `printMenu()` method we iterate over the `menus` aggregate and call `printMenu(Iterator<?>)` on the items:

{% highlight java %}
public class Waitress {
	ArrayList<Menu> menus;
     
  
	public Waitress(ArrayList<Menu> menus) {
		this.menus = menus;
	}
   
	public void printMenu() {
		Iterator<?> menuIterator = menus.iterator();
		
		System.out.print(MENU\n----\n);
		while(menuIterator.hasNext()) {
			Menu menu = (Menu)menuIterator.next();
			System.out.print("\n" + menu.getName() + "\n");
			printMenu(menu.createIterator());
		}
	}
   
	void printMenu(Iterator<?> iterator) {
		while (iterator.hasNext()) {
			MenuItem menuItem = (MenuItem)iterator.next();
			System.out.print(menuItem.getName() + ", ");
			System.out.print(menuItem.getPrice() + " -- ");
			System.out.println(menuItem.getDescription());
		}
	}
}  
{% endhighlight %}

To test this program we use the following snippet:

{% highlight java %}
public class MenuTestDrive {
	public static void main(String args[]) {
		PancakeHouseMenu pancakeHouseMenu = new PancakeHouseMenu();
		DinerMenu dinerMenu = new DinerMenu();
		ArrayList<Menu> menus = new ArrayList<Menu>();
		menus.add(pancakeHouseMenu);
		menus.add(dinerMenu);
		Waitress waitress = new Waitress(menus);
		waitress.printMenu();

	}
}
{% endhighlight %}


The output printing the menus is:

{% highlight bash %}
$ java MenuTestDrive
MENU
----
BREAKFAST
K&B's Pancake Breakfast, 2.99 -- Pancakes with scrambled eggs, and toast
Regular Pancake Breakfast, 2.99 -- Pancakes with fried eggs, sausage
Blueberry Pancakes, 3.49 -- Pancakes made with fresh blueberries, and blueberry syrup
Waffles, 3.59 -- Waffles, with your choice of blueberries or strawberries

LUNCH
Vegetarian BLT, 2.99 -- (Fakin') Bacon with lettuce & tomato on whole wheat
BLT, 2.99 -- Bacon with lettuce & tomato on whole wheat
Soup of the day, 3.29 -- Soup of the day, with a side of potato salad
Hotdog, 3.05 -- A hot dog, with saurkraut, relish, onions, topped with cheese
Steamed Veggies and Brown Rice, 3.99 -- Steamed vegetables over brown rice
Pasta, 3.89 -- Spaghetti with Marinara Sauce, and a slice of sourdough bread
{% endhighlight %}

This example will be improved in the next pattern: [The Composite Pattern](/design-patterns/composite).
