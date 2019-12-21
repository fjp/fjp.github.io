---
layout: single #collection
title: The Factory Pattern
permalink: /design-patterns/factory
excerpt: "The factory design pattern summarized."
date: 2018-09-22 15:41:35 +0200
categories: [programming, design patterns]
tags: [programming, design patterns, factory, pattern, creational]
comments: true
use_math: true
toc: false
# toc_label: "Unscented Kalman Filter"
classes: wide
header:
  teaser: /assets/pages/design-patterns/factory-method-pattern.png
  overlay_image: /assets/pages/design-patterns/factory-method-pattern.png
  overlay_filter: 0.5 # same as adding an opacity of 0.5 to a black background
  #caption: "Source: [**hpiracing**](http://www.hpiracing.com/de/kit/114343)"
  #show_overlay_excerpt: true
redirect_from:
    - /design-patterns/
sidebar:
    nav: "design-patterns"
author_profile: false
---


All factory patterns encapsulate object creation and there are two ways to use this pattern.

- [Factory Method](#factory-method)
- [Abstract Factory](#abstract-factory)

## Factory Method

<p>
<b>The Factory Method Pattern</b>  defines an interface
for creating an object, but lets subclasses decide which
class to instantiate. Factory Method lets a class defer
instantiation to subclasses.
</p>
{: .notice}

In this pattern an abstract creator class defines an abstract factory method that the
subclasses implement to produce products. Often the creator contains code that
depends on an abstract product, which is produced by a subclass. The creator
never really knows which concrete product was produced. Therefore, the intent of factory method
is to allow a class to defer instantiation to its subclasses.

Factory Method relies on inheritance: object creation is delegated to subclasses,
which implement the factory method to create objects.

<figure>
    <a href="/assets/pages/design-patterns/factory-method-pattern.png"><img src="/assets/pages/design-patterns/factory-method-pattern.png"></a>
    <figcaption>Decoupling object creation from baseclass to subclasses with the factory method pattern.</figcaption>
</figure>

In the example the abstract creator is `PizzaStore`.
This class contains the abstract factory method `abstract Pizza createPizza(String item)` that all concrete classes must implement.
Beside this abstract method this class can contain additional methods that manipulate a product, `Pizza orderPizza(String type)` in this example.

{% highlight java %}
public abstract class PizzaStore {

	abstract Pizza createPizza(String item);

	public Pizza orderPizza(String type) {
		Pizza pizza = createPizza(type);
		System.out.println("--- Making a " + pizza.getName() + " ---");
		pizza.prepare();
		pizza.bake();
		pizza.cut();
		pizza.box();
		return pizza;
	}
}
{% endhighlight %}

Next, follow two implementations of the abstract `PizzaStore` class.
Both of these implementations implement the abstract factory method, `createPizza()` in this case.

{% highlight java %}
public class ChicagoPizzaStore extends PizzaStore {

	Pizza createPizza(String item) {
        	if (item.equals("cheese")) {
            		return new ChicagoStyleCheesePizza();
        	} else if (item.equals("veggie")) {
        	    	return new ChicagoStyleVeggiePizza();
        	} else if (item.equals("clam")) {
        	    	return new ChicagoStyleClamPizza();
        	} else if (item.equals("pepperoni")) {
            		return new ChicagoStylePepperoniPizza();
        	} else return null;
	}
}
{% endhighlight %}



{% highlight java %}
public class NYPizzaStore extends PizzaStore {

	Pizza createPizza(String item) {
		if (item.equals("cheese")) {
			return new NYStyleCheesePizza();
		} else if (item.equals("veggie")) {
			return new NYStyleVeggiePizza();
		} else if (item.equals("clam")) {
			return new NYStyleClamPizza();
		} else if (item.equals("pepperoni")) {
			return new NYStylePepperoniPizza();
		} else return null;
	}
}
{% endhighlight %}

All the products must implement an interface which can be used by the creator and other classes that use the product.

{% highlight java %}
import java.util.ArrayList;

public abstract class Pizza {
	String name;
	String dough;
	String sauce;
	ArrayList<String> toppings = new ArrayList<String>();

	void prepare() {
		System.out.println("Prepare " + name);
		System.out.println("Tossing dough...");
		System.out.println("Adding sauce...");
		System.out.println("Adding toppings: ");
		for (String topping : toppings) {
			System.out.println("   " + topping);
		}
	}

	void bake() {
		System.out.println("Bake for 25 minutes at 350");
	}

	void cut() {
		System.out.println("Cut the pizza into diagonal slices");
	}

	void box() {
		System.out.println("Place pizza in official PizzaStore box");
	}

	public String getName() {
		return name;
	}

	public String toString() {
		StringBuffer display = new StringBuffer();
		display.append("---- " + name + " ----\n");
		display.append(dough + "\n");
		display.append(sauce + "\n");
		for (String topping : toppings) {
			display.append(topping + "\n");
		}
		return display.toString();
	}
}
{% endhighlight %}

This abstract product is implemented in the following two examples.

{% highlight java %}
public class NYStyleCheesePizza extends Pizza {

	public NYStyleCheesePizza() {
		name = "NY Style Sauce and Cheese Pizza";
		dough = "Thin Crust Dough";
		sauce = "Marinara Sauce";

		toppings.add("Grated Reggiano Cheese");
	}
}
{% endhighlight %}


{% highlight java %}
public class NYStyleClamPizza extends Pizza {

	public NYStyleClamPizza() {
		name = "NY Style Clam Pizza";
		dough = "Thin Crust Dough";
		sauce = "Marinara Sauce";

		toppings.add("Grated Reggiano Cheese");
		toppings.add("Fresh Clams from Long Island Sound");
	}
}
{% endhighlight %}


{% highlight java %}
public class ChicagoStylePepperoniPizza extends Pizza {
	public ChicagoStylePepperoniPizza() {
		name = "Chicago Style Pepperoni Pizza";
		dough = "Extra Thick Crust Dough";
		sauce = "Plum Tomato Sauce";

		toppings.add("Shredded Mozzarella Cheese");
		toppings.add("Black Olives");
		toppings.add("Spinach");
		toppings.add("Eggplant");
		toppings.add("Sliced Pepperoni");
	}

	void cut() {
		System.out.println("Cutting the pizza into square slices");
	}
}
{% endhighlight %}

{% highlight java %}
public class ChicagoStyleVeggiePizza extends Pizza {
	public ChicagoStyleVeggiePizza() {
		name = "Chicago Deep Dish Veggie Pizza";
		dough = "Extra Thick Crust Dough";
		sauce = "Plum Tomato Sauce";

		toppings.add("Shredded Mozzarella Cheese");
		toppings.add("Black Olives");
		toppings.add("Spinach");
		toppings.add("Eggplant");
	}

	void cut() {
		System.out.println("Cutting the pizza into square slices");
	}
}
{% endhighlight %}

The `PizzaTestDrive` shows an example how to use the Factory Method.

{% highlight java %}
public class PizzaTestDrive {

	public static void main(String[] args) {
		PizzaStore nyStore = new NYPizzaStore();
		PizzaStore chicagoStore = new ChicagoPizzaStore();

		Pizza pizza = nyStore.orderPizza("cheese");
		System.out.println("First order was a " + pizza.getName() + "\n");

		pizza = nyStore.orderPizza("cheese");
		System.out.println("Second order was a " + pizza.getName() + "\n");
	}
}
{% endhighlight %}

Running this program results in the following output.

{% highlight java %}
$java PizzaTestDrive

Preparing NY Style Sauce and Cheese Pizza
Tossing dough...
Adding sauce...
Adding toppings:
 Grated Regiano cheese
Bake for 25 minutes at 350
Cutting the pizza into diagonal slices
Place pizza in official PizzaStore box
First order was a NY Style Sauce and Cheese Pizza

Preparing Chicago Style Deep Dish Cheese Pizza
Tossing dough...
Adding sauce...
Adding toppings:
 Shredded Mozzarella Cheese
Bake for 25 minutes at 350
Cutting the pizza into square slices
Place pizza in official PizzaStore box
Second order was a Chicago Style Deep Dish Cheese Pizza
{% endhighlight %}


## Abstract Factory

<p>
<b>The Abstract Factory Pattern</b> provides an interface
for creating families of related or dependent objects
without specifying their concrete classes.
</p>
{: .notice}

Abstract Factory relies on object composition: object creation is implemented
in methods exposed in the factory interface. The intent of Abstract Factory is
to create families of related objects without having to depend on their concrete classes.

<figure>
    <a href="/assets/pages/design-patterns/abstract-factory-pattern.png"><img src="/assets/pages/design-patterns/abstract-factory-pattern.png"></a>
    <figcaption>Decoupling object creation from baseclass to subclasses with the factory method pattern.</figcaption>
</figure>



## Example using Factory Method and Abstract Factory

In this example the previous example of the [Factory Method](#factory-method)
gets extended to show how an Abstract Factory works.

The `public interface PizzaIngredientFactory` is implemented as an
abstract factory that can create families of products (the ingredients).
Methods to create products in an Abstract Factory are often implemented with
a Factory Method.

{% highlight java %}
public interface PizzaIngredientFactory {

	public Dough createDough();
	public Sauce createSauce();
	public Cheese createCheese();
	public Veggies[] createVeggies();
	public Pepperoni createPepperoni();
	public Clams createClam();

}
{% endhighlight %}

Each product has its own interface

{% highlight java %}
public interface Dough {
	public String toString();
}
{% endhighlight %}


{% highlight java %}
public interface Cheese {
	public String toString();
}
{% endhighlight %}


{% highlight java %}
public interface Sauce {
	public String toString();
}
{% endhighlight %}

Concrete implementations of these ingredient interface follow in the next code snippets.


{% highlight java %}
public class ThickCrustDough implements Dough {
	public String toString() {
		return "ThickCrust style extra thick crust dough";
	}
}
{% endhighlight %}

{% highlight java %}
public class ThinCrustDough implements Dough {
	public String toString() {
		return "Thin Crust Dough";
	}
}
{% endhighlight %}

{% highlight java %}
public class ReggianoCheese implements Cheese {

	public String toString() {
		return "Reggiano Cheese";
	}
}
{% endhighlight %}

{% highlight java %}
public class MozzarellaCheese implements Cheese {

	public String toString() {
		return "Shredded Mozzarella";
	}
}
{% endhighlight %}

{% highlight java %}
public class MarinaraSauce implements Sauce {
	public String toString() {
		return "Marinara Sauce";
	}
}
{% endhighlight %}

{% highlight java %}
public class PlumTomatoSauce implements Sauce {
	public String toString() {
		return "Tomato sauce with plum tomatoes";
	}
}
{% endhighlight %}

Each concrete subclass of the abstract factory class, `IngredientFactory`, creates a family of products.

{% highlight java %}
public class ChicagoPizzaIngredientFactory
	implements PizzaIngredientFactory
{

	public Dough createDough() {
		return new ThickCrustDough();
	}

	public Sauce createSauce() {
		return new PlumTomatoSauce();
	}

	public Cheese createCheese() {
		return new MozzarellaCheese();
	}

	public Veggies[] createVeggies() {
		Veggies veggies[] = { new BlackOlives(),
		                      new Spinach(),
		                      new Eggplant() };
		return veggies;
	}

	public Pepperoni createPepperoni() {
		return new SlicedPepperoni();
	}

	public Clams createClam() {
		return new FrozenClams();
	}
}
{% endhighlight %}

Here is another subclass that implements the abstract factory and therefore creates another family of products.

{% highlight java %}
public class NYPizzaIngredientFactory implements PizzaIngredientFactory {

	public Dough createDough() {
		return new ThinCrustDough();
	}

	public Sauce createSauce() {
		return new MarinaraSauce();
	}

	public Cheese createCheese() {
		return new ReggianoCheese();
	}

	public Veggies[] createVeggies() {
		Veggies veggies[] = { new Garlic(), new Onion(), new Mushroom(), new RedPepper() };
		return veggies;
	}

	public Pepperoni createPepperoni() {
		return new SlicedPepperoni();
	}

	public Clams createClam() {
		return new FreshClams();
	}
}
{% endhighlight %}

The creators, in this example the `PizzaStore` implementations instantiate the abstract factory classes.

{% highlight java %}
public class NYPizzaStore extends PizzaStore {

	protected Pizza createPizza(String item) {
		Pizza pizza = null;
		PizzaIngredientFactory ingredientFactory =
			new NYPizzaIngredientFactory();

		if (item.equals("cheese")) {

			pizza = new CheesePizza(ingredientFactory);
			pizza.setName("New York Style Cheese Pizza");

		} else if (item.equals("veggie")) {

			pizza = new VeggiePizza(ingredientFactory);
			pizza.setName("New York Style Veggie Pizza");

		} else if (item.equals("clam")) {

			pizza = new ClamPizza(ingredientFactory);
			pizza.setName("New York Style Clam Pizza");

		} else if (item.equals("pepperoni")) {

			pizza = new PepperoniPizza(ingredientFactory);
			pizza.setName("New York Style Pepperoni Pizza");

		}
		return pizza;
	}
}
{% endhighlight %}


{% highlight java %}
public class ChicagoPizzaStore extends PizzaStore {

	protected Pizza createPizza(String item) {
		Pizza pizza = null;
		PizzaIngredientFactory ingredientFactory =
		new ChicagoPizzaIngredientFactory();

		if (item.equals("cheese")) {

			pizza = new CheesePizza(ingredientFactory);
			pizza.setName("Chicago Style Cheese Pizza");

		} else if (item.equals("veggie")) {

			pizza = new VeggiePizza(ingredientFactory);
			pizza.setName("Chicago Style Veggie Pizza");

		} else if (item.equals("clam")) {

			pizza = new ClamPizza(ingredientFactory);
			pizza.setName("Chicago Style Clam Pizza");

		} else if (item.equals("pepperoni")) {

			pizza = new PepperoniPizza(ingredientFactory);
			pizza.setName("Chicago Style Pepperoni Pizza");

		}
		return pizza;
	}
}
{% endhighlight %}

All these changes require that the product interface itself can use the ingredients from the different factories.
The `prepare` method is now abstract which will get implemented in the different kinds pizzas.
This function uses the ingredients making use of the ingredient factory.  

{% highlight java %}
public abstract class Pizza {
	String name;

	Dough dough;
	Sauce sauce;
	Veggies veggies[];
	Cheese cheese;
	Pepperoni pepperoni;
	Clams clam;

	abstract void prepare();

	void bake() {
		System.out.println("Bake for 25 minutes at 350");
	}

	void cut() {
		System.out.println("Cutting the pizza into diagonal slices");
	}

	void box() {
		System.out.println("Place pizza in official PizzaStore box");
	}

	void setName(String name) {
		this.name = name;
	}

	String getName() {
		return name;
	}

	public String toString() {
		StringBuffer result = new StringBuffer();
		result.append("---- " + name + " ----\n");
		if (dough != null) {
			result.append(dough);
			result.append("\n");
		}
		if (sauce != null) {
			result.append(sauce);
			result.append("\n");
		}
		if (cheese != null) {
			result.append(cheese);
			result.append("\n");
		}
		if (veggies != null) {
			for (int i = 0; i < veggies.length; i++) {
				result.append(veggies[i]);
				if (i < veggies.length-1) {
					result.append(", ");
				}
			}
			result.append("\n");
		}
		if (clam != null) {
			result.append(clam);
			result.append("\n");
		}
		if (pepperoni != null) {
			result.append(pepperoni);
			result.append("\n");
		}
		return result.toString();
	}
}
{% endhighlight %}


Every concrete pizza needs to be composed with a ingredient factory member.

{% highlight java %}
public class CheesePizza extends Pizza {
	PizzaIngredientFactory ingredientFactory;

	public CheesePizza(PizzaIngredientFactory ingredientFactory) {
		this.ingredientFactory = ingredientFactory;
	}

	void prepare() {
		System.out.println("Preparing " + name);
		dough = ingredientFactory.createDough();
		sauce = ingredientFactory.createSauce();
		cheese = ingredientFactory.createCheese();
	}
}
{% endhighlight %}



{% highlight java %}
public class PepperoniPizza extends Pizza {
	PizzaIngredientFactory ingredientFactory;

	public PepperoniPizza(PizzaIngredientFactory ingredientFactory) {
		this.ingredientFactory = ingredientFactory;
	}

	void prepare() {
		System.out.println("Preparing " + name);
		dough = ingredientFactory.createDough();
		sauce = ingredientFactory.createSauce();
		cheese = ingredientFactory.createCheese();
		veggies = ingredientFactory.createVeggies();
		pepperoni = ingredientFactory.createPepperoni();
	}
}
{% endhighlight %}


{% highlight java %}
public class VeggiePizza extends Pizza {
	PizzaIngredientFactory ingredientFactory;

	public VeggiePizza(PizzaIngredientFactory ingredientFactory) {
		this.ingredientFactory = ingredientFactory;
	}

	void prepare() {
		System.out.println("Preparing " + name);
		dough = ingredientFactory.createDough();
		sauce = ingredientFactory.createSauce();
		cheese = ingredientFactory.createCheese();
		veggies = ingredientFactory.createVeggies();
	}
}
{% endhighlight %}


To create some products or pizzas the same test program can be executed listed above.
