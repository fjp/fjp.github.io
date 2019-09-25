---
layout: single #collection
title: The Singleton Pattern
permalink: /design-patterns/singleton
excerpt: "The singleton design pattern summarized."
date: 2018-09-20 15:41:35 +0200
categories: [programming, design patterns]
tags: [programming, design patterns, singleton, pattern, creational]
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
<b>The Singleton Pattern</b> ensures a class has only one
instance, and provides a global point of access to it.
</p>
{: .notice}

The Singleton pattern ensures you have at most one instance of a class in your application.
It provides a global access point to that instance. It is implemented with a private constructor, 
a static method to create one instance of your Singleton class and a static variable to check its existence.

The singleton pattern is taking on two responsibilities: 
It is not only responsible for managing its one instance (and providing global access), 
it is also responsible for whatever its main role is intended.
A singleton should not be inherited from because its constructor should remain private.

When designing multithreaded applications we need to examine the performance and resource constraints while 
carefully choosing an appropriate Singleton implementation. 
Beware of the double-checked locking implementation which can be not thread-safe.

The singleton is similar to global variables but without the downside of getting created at program start like global variables. Instead, the singleton can be created only when it is needed, which can avoid time consuming instantiation.
This is called **lazy instantiation**.

Examples where singletons are useful are:

- thread pools
- caches
- dialog boxes
- objects that handle preferences and registry settings
- objects used for logging
- objects that act as device drivers to devices like printers and graphics cards. 

In fact, for many of these types of objects, if we were to
instantiate more than one we'd run into all sorts of problems like incorrect
program behavior, overuse of resources, or inconsistent results.


<figure>
    <a href="/assets/pages/design-patterns/singleton-pattern.png"><img src="/assets/pages/design-patterns/singleton-pattern.png"></a>
    <figcaption>Singleton gives another method of creating unique objects with global access.</figcaption>
</figure>

In the following example is under construction.

{% highlight java %}

{% endhighlight %}

The concrete components implement this interface.

{% highlight java %}

{% endhighlight %}

Another concrete component.

{% highlight java %}

{% endhighlight %}



the following main class can be executed.

{% highlight java %}

{% endhighlight %}

Starting this application results in the following output.

{% highlight bash %}

{% endhighlight %}
