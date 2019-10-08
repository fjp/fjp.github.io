---
layout: single #collection
title: The Composite Pattern
permalink: /design-patterns/composite
excerpt: "The composite design pattern summarized."
date: 2019-10-08 15:41:35 +0200
categories: [programming, design patterns]
tags: [programming, design patterns, composite, pattern, structural]
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
<b>The Composite Pattern</b> allows you to compose objects into tree structures to represent 
part-whole hirarchies. Composite lets clients treat individual objects and compositions of objects uniformly.
</p>
{: .notice}

The Composite Pattern allows us to build structures of objects in the form of trees that contain both compositions of objects and individual objects as nodes. A composite contains components. Components come in two flavors: 
composites and leafe elements. This recursive structure contains composites that hold a set of children.
Those children may be other composites or leaf elements.

Using a composite structure, we can apply the same operations over both composites and individual objects. 
In other words, in most cases we can ignore the differences between compositions of objects and individual objects.

The [Iterator Pattern](/design-patterns/iterator) is commonly used with the Composite Pattern to iterate over its components.

<figure>
    <a href="/assets/pages/design-patterns/composite-pattern.png"><img src="/assets/pages/design-patterns/composite-pattern.png"></a>
    <figcaption>The Composite Pattern.</figcaption>
</figure>

A `Component` can implement default behavior for its `Leaf` and `Composite` subclasses. 
If a base class method is different in the subclasses a valid default implementation is to throw an exception.

The following example extends the previous example from the [Iterator Pattern](/design-patterns/iterator), which shows two restraunt menus, where both implement the same aggregate interface `Menu`. Each menu has menu items stored in different types of collections (aggregates such as `ArrayList`, standard array or `HashMap`). 
With the Iterator Pattern it is possible to iterate over these items without knowing the underlying type of the aggregate.


Let's start with the  `MenuComponent` class, which is the base class of `Leaf`, which describes the `MenuItem`, and `Composite`, which is a complete `Menu` in this example.

{% highlight java %}
public abstract class MenuComponent {
   
	public void add(MenuComponent menuComponent) {
		throw new UnsupportedOperationException();
	}
	public void remove(MenuComponent menuComponent) {
		throw new UnsupportedOperationException();
	}
	public MenuComponent getChild(int i) {
		throw new UnsupportedOperationException();
	}
  
	public String getName() {
		throw new UnsupportedOperationException();
	}
	public String getDescription() {
		throw new UnsupportedOperationException();
	}
	public double getPrice() {
		throw new UnsupportedOperationException();
	}
	public boolean isVegetarian() {
		throw new UnsupportedOperationException();
	}

	public abstract Iterator<MenuComponent> createIterator();
 
	public void print() {
		throw new UnsupportedOperationException();
	}
}
{% endhighlight %}

Each menu will have one or more menu items that implement the `MenuComponent` interface and play the role of Leafs in the composite:

{% highlight java %}
public class MenuItem extends MenuComponent {
 
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

	public Iterator<MenuComponent> createIterator() {
		return new NullIterator();
	}
 
	public void print() {
		System.out.print("  " + getName());
		if (isVegetarian()) {
			System.out.print("(v)");
		}
		System.out.println(", " + getPrice());
		System.out.println("     -- " + getDescription());
	}

}
{% endhighlight %}


As in the previous example, each aggregate implements the `createIterator()` method which is declared in the `Menu` interface. This time, the interface is a subclass of `MenuComponent` and acts as a Composite:

{% highlight java %}
import java.util.Iterator;
import java.util.ArrayList;

public class Menu extends MenuComponent {
	Iterator<MenuComponent> iterator = null;
	ArrayList<MenuComponent> menuComponents = new ArrayList<MenuComponent>();
	String name;
	String description;
  
	public Menu(String name, String description) {
		this.name = name;
		this.description = description;
	}
 
	public void add(MenuComponent menuComponent) {
		menuComponents.add(menuComponent);
	}
 
	public void remove(MenuComponent menuComponent) {
		menuComponents.remove(menuComponent);
	}
 
	public MenuComponent getChild(int i) {
		return menuComponents.get(i);
	}
 
	public String getName() {
		return name;
	}
 
	public String getDescription() {
		return description;
	}

  
	public Iterator<MenuComponent> createIterator() {
		if (iterator == null) {
			iterator = new CompositeIterator(menuComponents.iterator());
		}
		return iterator;
	}
 
 
	public void print() {
		System.out.print("\n" + getName());
		System.out.println(", " + getDescription());
		System.out.println("---------------------");
  
		Iterator<MenuComponent> iterator = menuComponents.iterator();
		while (iterator.hasNext()) {
			MenuComponent menuComponent = iterator.next();
			menuComponent.print();
		}
	}
}
{% endhighlight %}

All the client, in this example the `Waitress`, needs is the top-level menu component, that contains all the other menus, called `allmenus`: 

{% highlight java %}
public class Waitress {
	MenuComponent allMenus;
 
	public Waitress(MenuComponent allMenus) {
		this.allMenus = allMenus;
	}
 
	public void printMenu() {
		allMenus.print();
	}
}
{% endhighlight %}

To print the entire menu hierarchy, all the menus and all the menu items, we need to call `printMenu()` on the `Waitress`.


To define our menu we create the tree structure in the test program `MenuTestDrive`,
instead of having two menus that implement `Menu`. Here we first create all the menu objects including a top-level menu, 
named `allMenus`. The tree structure is created with the `Composite.add()` method. With this method it is also possible to add a menu to a menu because menus and menu items use the same `MenuComponent` interface.

{% highlight java %}
public class MenuTestDrive {
	public static void main(String args[]) {
		MenuComponent pancakeHouseMenu = 
			new Menu("PANCAKE HOUSE MENU", "Breakfast");
		MenuComponent dinerMenu = 
			new Menu("DINER MENU", "Lunch");
		MenuComponent cafeMenu = 
			new Menu("CAFE MENU", "Dinner");
		MenuComponent dessertMenu = 
			new Menu("DESSERT MENU", "Dessert of course!");
		MenuComponent coffeeMenu = new Menu("COFFEE MENU", "Stuff to go with your afternoon coffee");
  
		MenuComponent allMenus = new Menu("ALL MENUS", "All menus combined");
  
		allMenus.add(pancakeHouseMenu);
		allMenus.add(dinerMenu);
		allMenus.add(cafeMenu);
  
		pancakeHouseMenu.add(new MenuItem(
			"K&B's Pancake Breakfast", 
			"Pancakes with scrambled eggs, and toast", 
			true,
			2.99));
		pancakeHouseMenu.add(new MenuItem(
			"Regular Pancake Breakfast", 
			"Pancakes with fried eggs, sausage", 
			false,
			2.99));
		pancakeHouseMenu.add(new MenuItem(
			"Blueberry Pancakes",
			"Pancakes made with fresh blueberries, and blueberry syrup",
			true,
			3.49));
		pancakeHouseMenu.add(new MenuItem(
			"Waffles",
			"Waffles, with your choice of blueberries or strawberries",
			true,
			3.59));

		dinerMenu.add(new MenuItem(
			"Vegetarian BLT",
			"(Fakin') Bacon with lettuce & tomato on whole wheat", 
			true, 
			2.99));
		dinerMenu.add(new MenuItem(
			"BLT",
			"Bacon with lettuce & tomato on whole wheat", 
			false, 
			2.99));
		dinerMenu.add(new MenuItem(
			"Soup of the day",
			"A bowl of the soup of the day, with a side of potato salad", 
			false, 
			3.29));
		dinerMenu.add(new MenuItem(
			"Hotdog",
			"A hot dog, with saurkraut, relish, onions, topped with cheese",
			false, 
			3.05));
		dinerMenu.add(new MenuItem(
			"Steamed Veggies and Brown Rice",
			"Steamed vegetables over brown rice", 
			true, 
			3.99));
 
		dinerMenu.add(new MenuItem(
			"Pasta",
			"Spaghetti with Marinara Sauce, and a slice of sourdough bread",
			true, 
			3.89));
   
		dinerMenu.add(dessertMenu);
  
		dessertMenu.add(new MenuItem(
			"Apple Pie",
			"Apple pie with a flakey crust, topped with vanilla icecream",
			true,
			1.59));
  
		dessertMenu.add(new MenuItem(
			"Cheesecake",
			"Creamy New York cheesecake, with a chocolate graham crust",
			true,
			1.99));
		dessertMenu.add(new MenuItem(
			"Sorbet",
			"A scoop of raspberry and a scoop of lime",
			true,
			1.89));

		cafeMenu.add(new MenuItem(
			"Veggie Burger and Air Fries",
			"Veggie burger on a whole wheat bun, lettuce, tomato, and fries",
			true, 
			3.99));
		cafeMenu.add(new MenuItem(
			"Soup of the day",
			"A cup of the soup of the day, with a side salad",
			false, 
			3.69));
		cafeMenu.add(new MenuItem(
			"Burrito",
			"A large burrito, with whole pinto beans, salsa, guacamole",
			true, 
			4.29));

		cafeMenu.add(coffeeMenu);

		coffeeMenu.add(new MenuItem(
			"Coffee Cake",
			"Crumbly cake topped with cinnamon and walnuts",
			true,
			1.59));
		coffeeMenu.add(new MenuItem(
			"Bagel",
			"Flavors include sesame, poppyseed, cinnamon raisin, pumpkin",
			false,
			0.69));
		coffeeMenu.add(new MenuItem(
			"Biscotti",
			"Three almond or hazelnut biscotti cookies",
			true,
			0.89));
 
		Waitress waitress = new Waitress(allMenus);
   
		waitress.printMenu();
	}
}
{% endhighlight %}

At the end of this test program, the complete tree structure is handed to the `Waitress` which calls `printMenu()` that produces the following result:


{% highlight bash %}
$ java MenuTestDrive

ALL MENUS, All menus combined
---------------------

PANCAKE HOUSE MENU, Breakfast
---------------------
  K&B's Pancake Breakfast(v), 2.99
     -- Pancakes with scrambled eggs, and toast
  Regular Pancake Breakfast, 2.99
     -- Pancakes with fried eggs, sausage
  Blueberry Pancakes(v), 3.49
     -- Pancakes made with fresh blueberries, and blueberry syrup
  Waffles(v), 3.59
     -- Waffles, with your choice of blueberries or strawberries

DINER MENU, Lunch
---------------------
  Vegetarian BLT(v), 2.99
     -- (Fakin') Bacon with lettuce & tomato on whole wheat
  BLT, 2.99
     -- Bacon with lettuce & tomato on whole wheat
  Soup of the day, 3.29
     -- A bowl of the soup of the day, with a side of potato salad
  Hotdog, 3.05
     -- A hot dog, with saurkraut, relish, onions, topped with cheese
  Steamed Veggies and Brown Rice(v), 3.99
     -- Steamed vegetables over brown rice
  Pasta(v), 3.89
     -- Spaghetti with Marinara Sauce, and a slice of sourdough bread

DESSERT MENU, Dessert of course!
---------------------
  Apple Pie(v), 1.59
     -- Apple pie with a flakey crust, topped with vanilla icecream
  Cheesecake(v), 1.99
     -- Creamy New York cheesecake, with a chocolate graham crust
  Sorbet(v), 1.89
     -- A scoop of raspberry and a scoop of lime

CAFE MENU, Dinner
---------------------
  Veggie Burger and Air Fries(v), 3.99
     -- Veggie burger on a whole wheat bun, lettuce, tomato, and fries
  Soup of the day, 3.69
     -- A cup of the soup of the day, with a side salad
  Burrito(v), 4.29
     -- A large burrito, with whole pinto beans, salsa, guacamole

COFFEE MENU, Stuff to go with your afternoon coffee
---------------------
  Coffee Cake(v), 1.59
     -- Crumbly cake topped with cinnamon and walnuts
  Bagel, 0.69
     -- Flavors include sesame, poppyseed, cinnamon raisin, pumpkin
  Biscotti(v), 0.89
     -- Three almond or hazelnut biscotti cookies
{% endhighlight %}
