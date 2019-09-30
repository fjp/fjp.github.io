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

<figure>
    <a href="/assets/pages/design-patterns/adapter-pattern.png"><img src="/assets/pages/design-patterns/adapter-pattern.png"></a>
    <figcaption>Adapter.</figcaption>
</figure>

{% highlight java %}
public class Singleton {
	private static Singleton uniqueInstance;
 
	private Singleton() {}
 
	public static Singleton getInstance() {
		if (uniqueInstance == null) {
			uniqueInstance = new Singleton();
		}
		return uniqueInstance;
	}
 
	// other useful methods here
	public String getDescription() {
		return "I'm a classic Singleton!";
	}
}
{% endhighlight %}
The output of the thread safe implementation would be:

{% highlight bash %}
$ java SingletonClient
I'm a thread safe Singleton!
{% endhighlight %}
