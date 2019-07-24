---
layout: single #collection
title: Design Patterns
permalink: /design-patterns/
excerpt: "Awesome list of Design Patterns"
classes: wide
header:
  #overlay_image: /assets/projects/autonomous-rc-car/hpi-racing-bmw-m3.png
  #overlay_filter: 0.5 # same as adding an opacity of 0.5 to a black background
  #caption: "Source: [**hpiracing**](http://www.hpiracing.com/de/kit/114343)"
  #show_overlay_excerpt: true
sidebar:
  nav: "design-patterns"
author_profile: false
---

## Introduction

The following list shows some important design patterns, which are design principles to achieve great program architectures.


- [Strategy](/design-patterns/strategy)
- [Observer](/design-patterns/observer)
- [Decorator](/design-patterns/decorator)
- [Factory](/design-patterns/factory)


## SOLID Design Principles


<p>
<b>Single Responsibility Principle (SRP)</b> <br>
A class should only have a single responsibility.
</p>
{: .notice}

Example: Seperate `Journal` class and `PersistanceManager` class for saving the journal entries instead of putting the saving functionality into the `Journal` class itself.

<p>
<b>Open-Closed Principle (OCP)</b> <br>
Entities should be open for extension but closed for modification. It is better to extend a class (for example using 
multiple inheritance) rather than modifying a class that has already been tested (the change may be necessary due to changed requirements). Patterns that extend a class without modifying the class itself are the <a href="/observer/">observer pattern</a> and the <a href="/decorator/">decorator pattern</a> patterns.
</p>
{: .notice}

Example: `Product` class with different traits (color, size) and `ProductFilter` class that has methods to filter for items in a vector that contains pointers to products (`ProductFilter::by_size`, `ProductFilter::by_color`, `ProductFilter::by_color_and_size`). Adding more methods shows that the ProductFilter class would be modified, which should be avoided following the OCP. Instead, make a robust class, where the behavior can be changed dynamically. To solve this problem use for example the Specification Pattern (which is no GOF pattern). Create a specification interface `ISpecification` that has a single pure virtual function `bool is_satisfied(T item)` to check if an item satisfies a specification. Use anotehr interface for the Filter `IFilter` with a single pure virtual funciton `vector<T*> filter(vector<T*> items, ISpecification<T>& spec)`. A concrete class that implements the IFilter interface will loop through the items and use the passed specification to return only the items that satisfy the specification. The only thing that is missing, is to implement concrete specification classes that inherit from the ISpecification interface. For example color, size and composite types that have a constructor which initializes their members (color, size, or both) and implement `is_satisfied` by comparing the passed element with the member variable. Do not modify the original classes that are probably already tested, instead extend them using inheritance.

<p>
<b>Liskov Substitution Principle (LSP)</b> <br>
Objects should be replaceable with instances of their subtypes without altering the correctness of the program. To follow this principle use the <a href="/factory/">factory pattern</a> (factory method or abstract factory).
</p>
{: .notice}

Example: The classical rectangles and squares example shows how this principle is violated. Assume a `process(Rectangles& rect)` function which can output wrong results when called with a `Square` class that inherits from, or is a base class of `Rectangle`. To solve this use a factory `RectangleFactory` class that has methods to generate Rectangles (`Rectangle CreateRectangel(int width, int height)`, `Rectangle CreateSquare(int size)`).

<p>
<b>Interface Segregation Principle (ISP)</b> <br>
Many client-specific interfaces are better than one general purpose interface. No client (somebody how uses your code) should be forced to depend on methods that it does not use.
</p>
{: .notice}

<p>
<b>Dependency Inversion Principle (ISP)</b> <br>
Dependencies should be abstract rather than concrete. Depend upon abstractions. Do not depend upon concrete classes.
</p>
{: .notice}


## Further Design Principles

<p>
<b>Encapsulate what varies</b> <br>
Identify the aspects of your application that vary and separate them from
what stays the same.
</p>
{: .notice}

<p>
<b>Program to an interface, not an implementation. </b>
</p>
{: .notice}

<p>
<b>Favor composition over inheritance. </b>
With composition it is possible to delegate behaviors instead of inheriting it.
</p>
{: .notice}

<p>
  <b>Strive for loosely coupled designs between objects that interact. </b>
  Loosely coupled designs allow us to build flexible object oriented
systems that can handle change because they minimize
the interdependency between objects. This principle can be seen in the <a href="/observer/">observer pattern</a>.
</p>
{: .notice}

## Links

- [Boost Dependency Injection DI](https://boost-experimental.github.io/di/)

## References

{% raw %}
<a href="https://www.amazon.de/Head-First-Design-Patterns-Freeman/dp/0596007124/ref=as_li_ss_il?ie=UTF8&linkCode=li2&tag=fjp-21&linkId=38bf781d92136c82d722d01735b6f3df&language=de_DE" target="_blank">
<img border="0" src="//ws-eu.amazon-adsystem.com/widgets/q?_encoding=UTF8&ASIN=0596007124&Format=_SL160_&ID=AsinImage&MarketPlace=DE&ServiceVersion=20070822&WS=1&tag=fjp-21&language=de_DE" >
</a>
<img src="https://ir-de.amazon-adsystem.com/e/ir?t=fjp-21&language=de_DE&l=li2&o=3&a=0596007124" width="1" height="1" border="0" alt="" style="border:none !important; margin:0px !important;" />
{% endraw %}

{% raw %}
<a href="https://www.amazon.de/Design-Patterns-Elements-Reusable-Object-Oriented/dp/0201633612/ref=as_li_ss_il?crid=294S6ZO5GLTAX&keywords=gamma+design+patterns&qid=1563989351&s=gateway&sprefix=gamma+design,aps,230&sr=8-1&linkCode=li2&tag=fjp-21&linkId=845330f357bcc7b2eedce3e15a28c1b6&language=de_DE" target="_blank"><img border="0" src="//ws-eu.amazon-adsystem.com/widgets/q?_encoding=UTF8&ASIN=0201633612&Format=_SL160_&ID=AsinImage&MarketPlace=DE&ServiceVersion=20070822&WS=1&tag=fjp-21&language=de_DE" ></a><img src="https://ir-de.amazon-adsystem.com/e/ir?t=fjp-21&language=de_DE&l=li2&o=3&a=0201633612" width="1" height="1" border="0" alt="" style="border:none !important; margin:0px !important;" />
{% endraw %}
