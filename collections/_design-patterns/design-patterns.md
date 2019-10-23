---
layout: single #collection
title: Design Patterns
permalink: /design-patterns/
excerpt: "Awesome list of Design Patterns"
date: 2018-08-20 15:41:35 +0200
categories: [programming, design patterns]
tags: [programming, design patterns, solid, books]
toc: true
comments: true
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



<p>
<b>A Pattern</b> is a solution to a problem in a context.
</p>
{: .notice}

The following list shows some important design patterns, which are design principles to achieve great program architectures.

- [Strategy](/design-patterns/strategy)
- [Observer](/design-patterns/observer)
- [Decorator](/design-patterns/decorator)
- [Factory](/design-patterns/factory)
- [Singleton](/design-patterns/singleton)
- [Command](/design-patterns/command)
- [Adapter](/design-patterns/adapter)
- [Facade](/design-patterns/facade)
- [Template Method](/design-patterns/template-method)

## Classification of Design Patterns

Design patterns are partitioned into different classes. This helps to oragnize them, narrow our searches to a subset of all Design Patterns,
and make comparisons within a group of patterns. The most well-known scheme was used by the first [patterns catalog](/design-patterns/#patterns-catalog) and partitions patterns into three distinct categories based on their purposes: Creational, Behavioral, and Structural.
On the left you see the patterns partitioned into the classic scheme.

<p>
<b>Creational Patterns</b> involve object instantiation and all provide a way to decouple a client from the objects it needs to instantiate.
</p>
{: .notice}

<p>
<b>Structural Patterns</b> let you compose classes or objects into larger structures.
</p>
{: .notice}

<p>
Any pattern that is a <b>Behavioral Patterns</b> is concerned with how classes and objects interact and distribute responsibility.
</p>
{: .notice}

Patterns are often classified by a second attribute: whether or not the pattern deals with classes or objects:

<p>
<b>Class Patterns</b> describe how relationshiops between classes are defined via inheritance. 
Relationships in class patterns are established at compile time.
</p>
{: .notice}

<p>
<b>Object Patterns</b> describe relationships between objects and are primarily defined by composition. 
Relationships in object patterns are typically created at runtime and are more dynamic and flexible.
</p>
{: .notice}

Class patterns are for example: Template Method, Factory Method, Adapter and Interpreter. Most of the other patterns are Object patterns.

Categories give us a way to think about the way groups of patterns relate and how patterns within a group relate to one another. They also give us a way to extrapolate to new patterns.

## SOLID Design Principles

### Single Responsibility Principle (SRP)

<p>
<b>Single Responsibility Principle (SRP)</b> <br>
A class should only have a single responsibility.
</p>
{: .notice}

Example: Seperate `Journal` class and `PersistanceManager` class for saving the journal entries instead of putting the saving functionality into the `Journal` class itself.

### Open-Closed Principle (OCP)

<p>
<b>Open-Closed Principle (OCP)</b> <br>
Entities should be open for extension but closed for modification. It is better to extend a class (for example using 
multiple inheritance) rather than modifying a class that has already been tested (the change may be necessary due to changed requirements). Patterns that extend a class without modifying the class itself are the <a href="/observer/">observer pattern</a> and the <a href="/decorator/">decorator pattern</a> patterns.
</p>
{: .notice}

Example: `Product` class with different traits (color, size) and `ProductFilter` class that has methods to filter for items in a vector that contains pointers to products (`ProductFilter::by_size`, `ProductFilter::by_color`, `ProductFilter::by_color_and_size`). Adding more methods shows that the ProductFilter class would be modified, which should be avoided following the OCP. Instead, make a robust class, where the behavior can be changed dynamically. To solve this problem use for example the Specification Pattern (which is no GOF pattern). Create a specification interface `ISpecification` that has a single pure virtual function `bool is_satisfied(T item)` to check if an item satisfies a specification. Use anotehr interface for the Filter `IFilter` with a single pure virtual funciton `vector<T*> filter(vector<T*> items, ISpecification<T>& spec)`. A concrete class that implements the IFilter interface will loop through the items and use the passed specification to return only the items that satisfy the specification. The only thing that is missing, is to implement concrete specification classes that inherit from the ISpecification interface. For example color, size and composite types that have a constructor which initializes their members (color, size, or both) and implement `is_satisfied` by comparing the passed element with the member variable. Do not modify the original classes that are probably already tested, instead extend them using inheritance.

### Liskov Substitution Principle (LSP)

<p>
<b>Liskov Substitution Principle (LSP)</b> <br>
Objects should be replaceable with instances of their subtypes without altering the correctness of the program. To follow this principle use the <a href="/factory/">factory pattern</a> (factory method or abstract factory).
</p>
{: .notice}

Example: The classical rectangles and squares example shows how this principle is violated. Assume a `process(Rectangles& rect)` function which can output wrong results when called with a `Square` class that inherits from, or is a base class of `Rectangle`. To solve this use a factory `RectangleFactory` class that has methods to generate Rectangles (`Rectangle CreateRectangel(int width, int height)`, `Rectangle CreateSquare(int size)`).

### Interface Segregation Principle (ISP)

<p>
<b>Interface Segregation Principle (ISP)</b> <br>
Many client-specific interfaces are better than one general purpose interface. No client (somebody how uses your code) should be forced to depend on methods that it does not use. Patterns that use this principle are the <a href="/decorator/">decorator pattern</a>.
</p>
{: .notice}

Example: Break up an interface into multiple other interfaces. Assume a multi-function device that is able to print, scan and fax some `Documents`.  A violation would be to use a single interface `IMachine` which has pure virtual methods (`print(vector<Documents*> docs)`, `scan(vector<Documents*> docs)`, `fax(vector<Documents*> docs)`). A client that only wants to use some functionalities of this interface has to make an implementation `MFP` (Multi Functional Peripheral). This is a bad idea for two main reasons: everytime only parts of the functionality changes, all the other methods need to be recompiled too. Anotehr reason is that the client probably does not need all of the functionality but has to implement it using the `IMachine` interface. ISP is about breaking up the monolithic interface and create piece-wise abstractions. Therefore, create a spearate interface for the printing `IPrinter`, scanning `IScanner` and faxing `IFax` functionality. These interfaces implement only the functionality that is related to them. If a client requires a printer `Printer`, it has to implement the `IPrinter` interface and its method `print(vector<Documents*> docs)`. If a client needs the whole machine, you can use multiple inheritance `struct IMachine : IPrinter, IScanner`. An actual implementation would then result in the [decorator pattern](/design-patterns/decorator), which is an aggregate of the functionalities that it inherits: `Machine : IMachine` would have member variable references or shared pointers of type `IPrinter` and `IScanner` and a constructor that initializes these members (this is also an example of Dependency Injection). The actual implementations of `print` and `scan` methods would then proxy it over to the variable members and use their methods `IPrinter::print` and `IScanner::scan`. With this design it is possible to use different implementations of functionality that implment an interface (for example a color printer that implements `IPrinter`) and pass it to the constructor of the `Machine` struct, which would be an inversion of control container that is then reconfigured. Summary: Break up interfaces into small interfaces and to use a big interface that contains all parts use multiple inheritance. The concrete realisation would be a [decorator pattern](/design-patterns/decorator).

### Dependency Inversion Principle (DIP)

<p>
<b>Dependency Inversion Principle (DIP)</b> <br>
Dependencies should be abstract rather than concrete. Depend upon abstractions. Do not depend upon concrete classes.
</p>
{: .notice}

Summary: 
1. High-level modules should not depend on low-level modules. Both should depend upon abstraction. 
Example: reporting component (high-level module) should depend on a `ConsoleLogger`, but can depend on an `ILogger`. A logging mechanism (low-level module) should also depend upon an abstraction. This makes it easier to substitue during runtime and for testing.
2. Abstraction should not depend upon details. Details should depend upon abstractions: Dependencies on interfaces and supertypes (base classes) is better than dependencies on concrete types. This is because it is hard to substitue afterwards if a concrete type is used (for example as a function argument). Compare this with the LSP (substitue a subtype with a supertype). 

Terms:
1. Inversion of Control (IoC): the actual process of creating abstractions and getting them to replace dependencies.
2. Dependency Injection: use of software frameworks ([boost di](https://boost-experimental.github.io/di/)) to ensure that a component's dependencies are satisfied. These dependencies don't have to be concrete types but can be interfaces or supertypes. These can be substitute with the subtypes as they are configured in the inversion of control container. The Inversion of Control Container manages the dependency injection. Injection means that the Inversion of Control Container initializes dependencies of a particular class by passing them as constructor parameters.

Example: Assume a car class `Car` that has a shared pointer member that points to an egnine class `Engine`, which is initialized by passing an engine object as a parameter to the constructor. This means that the engine is injected into the car, which it then depends upon. 
In case there are other dependencies that need to be set by passing them to the constructor, all of them need to be instantiated before. A dependency injection (di) library such as [boost di](https://boost-experimental.github.io/di/) can avoid this tedious task. This is done using an injector that initializes all dependencies and injects them into the `Car` class. A di library does this by checking the arguments of the constructor. 

Another example would be an `ILogger` interface for logging that has a virtual `LOG(const std::string& s)` method. A concrete class `ConsoleLogger : ILogger`. An injector can be bound using `bind` to use a concrete ConsoleLogger when a client requests an `ILogger`. 

## Further Design Principles

<p>
<b>Encapsulate what varies</b> <br>
Identify the aspects of your application that vary and separate them from
what stays the same.
</p>
{: .notice}

<p>
<b>Favor composition over inheritance.</b>
With composition it is possible to delegate behaviors instead of inheriting them.
</p>
{: .notice}

<p>
<b>Program to an interface, not an implementation. </b>
</p>
{: .notice}

<p>
  <b>Strive for loosely coupled designs between objects that interact. </b>
  Loosely coupled designs allow us to build flexible object oriented
systems that can handle change because they minimize
the interdependency between objects. This principle can be seen in the <a href="/observer/">observer pattern</a>.
</p>
{: .notice}

<p>
<b>Classes should be open for extension but closed for modification.</b>
Instead of modifying a class by adding new methods and members use inheritance or even better composition. 
Extend the class by adding a new interface class member. This makes it possible to change the behavior even at runtime.
</p>
{: .notice}

<p>
<b>Depend on abstractions. Do not depend on concrete classes.</b>
</p>
{: .notice}

<p>
<b>Talk only to your friends.</b>
This principle is important for maintining a low level of coupling between multiple classes.
It prevents us from creating designs that have a larage number of classes coupled together so 
that changes in one part of the system cascade to other parts.
Having dependencies between many classes, results in a fragile system that will be costly to maintain and complex for others to understand.
</p>
{: .notice}

To follow this principle the following guidlines should be met: regarding any method of an object, the method should only invoke methods that belong to:

- The object itself
- Objects passed in as a parameter to the method
- Any object the method creates or instantiates
- Any components of the object

Calling methods on objects that were returned from calling other methods would violate this principle as the following example shows:

{% highlight java %}
public float getTemp() {
    Thermometer thremometer = station.getThermometer();
    return thermometer.getTemperature();
}
{% endhighlight %}

To avoid violating the principle and keeping the dependencies low, it is possible to delegate the request to the object 
directly without the need to call methods on a returned object:

{% highlight java %}
public float getTemp() {
    return station.getTemperature();
}
{% endhighlight %}

This however, implies that the object provides this method for us.

While the principle reduces dependencies between objects and maintenance, 
one disadvantage of this principle is that more "wrapper" classes are written to handle method calls to other components. 
This can result in increased complexity and development time as well as decreased runtime performance.


<p>
<b>The Hollywood Principle.</b>
Don't call us, we'll call you.
</p>
{: .notice}

This principle gives a way to prevent "dependency rot", which happens when using interweaving dependencies between high- and low-level components. The principle guides us to put decision making in high-level modules that can decide how and when to call low-level modules. With the Hollywood Principle, we allow low-level components to hook themselves into a system, but the high-level components determine when they are needed, and how. The low-level never call a high-level component directly.
It gives a technique for creating designs that allow low-level structures to interoperate while preventing other classes from becoming too dependent on them. A low-level component is allowd to call a method of a high-level component, for example when using an inherited method up in the hierarchy. The goal is to avoid creating explicit circular dependencies between the low-lefvel and the high-level components.

Like the Dependency Inversion Principle, the Hollywood Principle has the goal of decoupling. It provides a technique for building frameworks or components so that lower-level components can be hooked into the computation, but without creating dependencies between the lower-level coponentnts and the hight-level layers. 

Patterns that make use of the Hollywood Principle are: [Factory Method](/design-patterns/factory), [Observer](/design-patterns/observer) and [Template Method](/design-patterns/template-method).

<p>
<b>Single Responsibility</b> 
A class should have only one reason to change.
</p>
{: .notice}

Every responsibility of a class is an area of potential change. More than one responsibility means more than one area of change. This principle guides us to keep each class to a single responsibility. **Cohesion** is a term used to as a measure of how closely a class or a module supports a single purpose or responsibility. Cohesion is a more general concept than the Single Responsibility Principle, but the two are closely related. Classes that adhere to the principle tend to have high cohesion and are more maintainable than classes that take on multiple responsibilities and have low cohesion.

## Patterns Catalog

A patterns catalog takes a set of patterns and describes each in detail along with its relationship to the other patterns.
The first and most definitive patterns catalog is Design Patterns: Elements of Reusable Object-Oriented Software, by Gamma, Helm, 
Johnson & Vlissides (Addison Wesley). This catalog lays out 23 fundamental patterns.

Every pattern follows a template. For each pattern a patterns catalog defines the name of the pattern and has a few sections that 
tell us more about the pattern. For instance, there is an **Intent** section that describes what the pattern is, kind of like a definition. Then there are **Motivation** and **Applicability** sections that describe when and where the pattern might be used.

To use the patterns catalog, get familiar with all the patterns and their relationships first. This way, you already have an idea
which pattern could provide a suitable solution for a specific problem in a context. 
Look at the **Motivation** and **Applicability** sections to make sure You've got it right. 
There is also another really important section: **Consequences** to make sure there won't be some unintended effect on the intended design.

Once we know that a pattern is suitable it needs to be integrated into our currrent design, which requires implementing it correctly.
That's where the class diagram comes in. First read over the **Structure** section to review the diagram and then over the **Participants** section to make sure you understand each class's role. From there, work it into your design, making any alterations you need to make it fit. Then review the **Implementation and Sample Code** sections to make sure you know about any good implementation techniques or gotchas you might encounter.

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

The code examples for the single patterns are from the book [Head First Design Patterns](https://amzn.to/2lJjduN) and can be found on their [GitHub page](https://github.com/bethrobson/Head-First-Design-Patterns).
