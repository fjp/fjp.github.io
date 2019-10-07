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


<figure>
    <a href="/assets/pages/design-patterns/iterator-pattern.png"><img src="/assets/pages/design-patterns/iterator-pattern.png"></a>
    <figcaption>Iterator Pattern uses a Factory Method `createIterator()` to create an iterator for an Aggregate.</figcaption>
</figure>

We distinguish between "internal" and "external" iterators.
Using an external iterator, the client controls the iteration by calling `next()` to get the next element. An internal iterator is controlled by the iterator iself. This way, the iterator is stepping through the elements and thereby controls 
the iteration itself. To get apply operations on the elements we have to pass the internal iterator a method. With internal iterators the client doesn't have control of the iteration, which might not be required if a single operation should be applied to all elements.


{% highlight java %}

{% endhighlight %}



{% highlight java %}

{% endhighlight %}

The:

{% highlight java %}

{% endhighlight %}

The 


{% highlight java %}

{% endhighlight %}


The output of the program 

{% highlight bash %}
$ java 
{% endhighlight %}
