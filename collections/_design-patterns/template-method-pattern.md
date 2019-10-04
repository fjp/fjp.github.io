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

The Template Method Pattern 

<figure>
    <a href="/assets/pages/design-patterns/template-method-pattern.png"><img src="/assets/pages/design-patterns/template-method-pattern.png"></a>
    <figcaption>Template Method Pattern.</figcaption>
</figure>


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


{% highlight java %}

{% endhighlight %}

Each of 

{% highlight java %}

{% endhighlight %}


{% highlight java %}

{% endhighlight %}



{% highlight java %}

{% endhighlight %}


The output of the program is:

{% highlight bash %}
$ java 
{% endhighlight %}
