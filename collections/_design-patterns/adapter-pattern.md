---
layout: single #collection
title: The Adapter Pattern
permalink: /design-patterns/adapter
excerpt: "The adapter design pattern summarized."
date: 2019-09-30 15:41:35 +0200
categories: [programming, design patterns]
tags: [programming, design patterns, adapter, pattern, structural]
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
<b>The Adapter Pattern</b> converts the interface of a class into
another interface the clients expect. Adapter lets classes work 
together that couldn't otherwise because of incompatible interfaces.
</p>
{: .notice}

The Adapter Pattern is used when an existing adaptee class is needed and its interface is not the one a client needs.
For this, the pattern changes an intefcace into one a client expects using a adapter class. 
To achieve this, there are two forms of the Adapter Pattern: object and class adapters. 
Class adapters require multiple inheritance. They inherit from the adaptee and the target interface, 
which is expected by the client. With the object adapter, the adapter implements the target interface and wraps the adaptee by holding an instance of it.

The client is implemented against the target interface and uses the adapter in the following way:

1. The client makes a request to the adapter by calling a method on it using the target interface.
2. The adapter translates the request into one or more calls on the adaptee using the adaptee interface
3. The client receives the results of the call and never knows there is an adapter doing the translation.

<figure>
    <a href="/assets/pages/design-patterns/adapter-object-pattern.png"><img src="/assets/pages/design-patterns/adapter-object-pattern.png"></a>
    <figcaption>Object Adapter Pattern.</figcaption>
</figure>

Adapters are similar to the [Facade Pattern](/design-patterns/facade) and the [Decorator Pattern](/design-patterns/decorator). An adapter wraps an object to change its interface, 
a decorator wraps an object to add new behaviors and responsibilities, 
and a facade "wraps" a set of objects to simplify.

{% highlight java %}

{% endhighlight %}

The output of the thread safe implementation would be:

{% highlight bash %}
$ java 
{% endhighlight %}
