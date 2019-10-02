---
layout: single #collection
title: The Facade Pattern
permalink: /design-patterns/facade
excerpt: "The facade design pattern summarized."
date: 2019-09-30 15:41:35 +0200
categories: [programming, design patterns]
tags: [programming, design patterns, facade, pattern, structural]
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
<b>The Facade Pattern</b> provides a unified interface to a set of interfaces ina subsystem.
Facade defines a higher-level interface that makes the subsystem easier to use.
</p>
{: .notice}

The Facade Pattern is used when you need to simplify and unify a large interface or complex set of interfaces. 
A facade decouples a client from a complex subsystem. To implement a facade we compose the facade with its subsystem
and use delegation to perform the work of the facade. For a complex subsystem it can be helpful to implement more than one facade.

<figure>
    <a href="/assets/pages/design-patterns/facade-pattern.png"><img src="/assets/pages/design-patterns/facade-pattern.png"></a>
    <figcaption>Facade Pattern implements the Adaptee and uses composition for the target interface.</figcaption>
</figure>

This pattern is similar to the [Adapter Pattern](/design-patterns/adapter) and the [Decorator Pattern](/design-patterns/decorator).
An adapter wraps an object to change its interface, a decorator wraps an object to add new behaviors and responsibilities, 
and a facade "wraps" a set of objects to unify and simplify.


Example:

{% highlight java %}

{% endhighlight %}

An implementation :

{% highlight java %}

{% endhighlight %}

The 

{% highlight java %}

{% endhighlight %}


The followign main program:

{% highlight java %}

{% endhighlight %}


The output of the program is:

{% highlight bash %}
$ java 
{% endhighlight %}
