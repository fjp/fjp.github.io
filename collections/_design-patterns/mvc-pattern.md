---
layout: single #collection
title: The Model View Controller Pattern
permalink: /design-patterns/mvc
excerpt: "The model view controller design pattern summarized."
date: 2019-10-11 15:41:35 +0200
categories: [programming, design patterns]
tags: [programming, design patterns, mvc, model, view, controller, pattern, architectural]
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
<b>The Model View Controller Pattern</b> .
</p>
{: .notice}


<figure>
    <a href="/assets/pages/design-patterns/mvc-pattern.png"><img src="/assets/pages/design-patterns/mvc-pattern.png"></a>
    <figcaption>The Model View Controller Pattern.</figcaption>
</figure>


## Example

The following example shows a music player that uses a `BeatModel` that contains the data, a controller to start and 
stop the sound playing and set the beats per minute. Two views are used, one to control the model and one to view
its state.

Let's start with the `BeatModelInterface`:

{% highlight java %}
public interface BeatModelInterface {
	void initialize();
  
	void on();
  
	void off();
  
    void setBPM(int bpm);
  
	int getBPM();
  
	void registerObserver(BeatObserver o);
  
	void removeObserver(BeatObserver o);
  
	void registerObserver(BPMObserver o);
  
	void removeObserver(BPMObserver o);
}
{% endhighlight %}


{% highlight java %}

{% endhighlight %}


Here is the output after running the two test programs:

{% highlight bash %}
$ java 

{% endhighlight %}
