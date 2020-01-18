---
layout: single #collection
title: The Observer Pattern
permalink: /design-patterns/observer
excerpt: "The strategy design pattern summarized."
date: 2018-09-21 15:41:35 +0200
categories: [programming, design patterns]
tags: [programming, design patterns, observer, pattern, behavioral]
comments: true
use_math: true
toc: false
# toc_label: "Unscented Kalman Filter"
classes: wide
header:
  teaser: /assets/pages/design-patterns/observer-pattern.png
  overlay_image: /assets/pages/design-patterns/observer-pattern.png
  #overlay_filter: 0.5 # same as adding an opacity of 0.5 to a black background
  caption: "Source: [**Head First Design Patterns**]({{ page.url }}/#reference)"
  #show_overlay_excerpt: true
redirect_from:
    - /design-patterns/
sidebar:
    nav: "design-patterns"
author_profile: false
---

<p>
<b>The Observer Pattern</b> defines a one-to-many
dependency between objects so that when one
object changes state, all of its dependents are
notified and updated automatically.
</p>
{: .notice}

The observer pattern consists of publishers and subscribers similar to a newspaper subscription,
where the publisher is also known as subject or observable and the subscribers are called observers.

The observer pattern provides an object design where subjects and observers are
loosely coupled because of the following reasons:
- The only thing the subject knows about an observer is that it
implements a certain interface (the Observer interface).
- We can add new observers at any time.
- We never need to modify the subject to add new types of observers.
- We can reuse subjects or observers independently of each other.
- Changes to either the subject or an observer will not affect the other.

With the observer pattern it is possible to pull or push data from the
observable whereas pull is considered more correct.

It is also important to note that you should not depend on a specific order of
notification for your observers.

<figure>
    <a href="/assets/pages/design-patterns/observer-pattern.png"><img src="/assets/pages/design-patterns/observer-pattern.png"></a>
    <figcaption>Loosely coupled objects with the observer pattern.</figcaption>
</figure>

Here is the interface of a subject that a concrete subject class needs to implement in order to
register, remove and notify observers.

{% highlight java %}
public interface Subject {
	public void registerObserver(Observer o);
	public void removeObserver(Observer o);
	public void notifyObservers();
}
{% endhighlight %}

As an example for a concrete subject the following weather data class is given.
It has a member `ArrayList<Observer>` that stores the currently registered observers.

{% highlight java %}
import java.util.*;

public class WeatherData implements Subject {
	private ArrayList<Observer> observers;
	private float temperature;
	private float humidity;
	private float pressure;

	public WeatherData() {
		observers = new ArrayList<Observer>();
	}

	public void registerObserver(Observer o) {
		observers.add(o);
	}

	public void removeObserver(Observer o) {
		int i = observers.indexOf(o);
		if (i >= 0) {
			observers.remove(i);
		}
	}

	public void notifyObservers() {
		for (Observer observer : observers) {
			observer.update(temperature, humidity, pressure);
		}
	}

	public void measurementsChanged() {
		notifyObservers();
	}

	public void setMeasurements(float temperature, float humidity, float pressure) {
		this.temperature = temperature;
		this.humidity = humidity;
		this.pressure = pressure;
		measurementsChanged();
	}

	public float getTemperature() {
		return temperature;
	}

	public float getHumidity() {
		return humidity;
	}

	public float getPressure() {
		return pressure;
	}

}
{% endhighlight %}

When the `notifyObservers()` function gets called, which happens when some data of
the subject changes, the subject knows how to inform the observers because they all
implement the same interface.

{% highlight java %}
public interface Observer {
	public void update(float temp, float humidity, float pressure);
}
{% endhighlight %}


The observers that implement this interface know about the subject, which allows them
to register themselves to be kept informed. This can be seen in the following
example that also implements another interface `DisplayElement`.


{% highlight java %}
public class CurrentConditionsDisplay implements Observer, DisplayElement {
	private float temperature;
	private float humidity;
	private Subject weatherData;

	public CurrentConditionsDisplay(Subject weatherData) {
		this.weatherData = weatherData;
		weatherData.registerObserver(this);
	}

	public void update(float temperature, float humidity, float pressure) {
		this.temperature = temperature;
		this.humidity = humidity;
		display();
	}

	public void display() {
		System.out.println("Current conditions: " + temperature
			+ "F degrees and " + humidity + "% humidity");
	}
}
{% endhighlight %}

Another example of a observer implementation.

{% highlight java %}
public class StatisticsDisplay implements Observer, DisplayElement {
	private float maxTemp = 0.0f;
	private float minTemp = 200;
	private float tempSum= 0.0f;
	private int numReadings;
	private WeatherData weatherData;

	public StatisticsDisplay(WeatherData weatherData) {
		this.weatherData = weatherData;
		weatherData.registerObserver(this);
	}

	public void update(float temp, float humidity, float pressure) {
		tempSum += temp;
		numReadings++;

		if (temp > maxTemp) {
			maxTemp = temp;
		}

		if (temp < minTemp) {
			minTemp = temp;
		}

		display();
	}

	public void display() {
		System.out.println("Avg/Max/Min temperature = " + (tempSum / numReadings)
			+ "/" + maxTemp + "/" + minTemp);
	}
}
{% endhighlight %}

And the additional interface they use.

{% highlight java %}
public interface DisplayElement {
	public void display();
}
{% endhighlight %}

The following main `WeatherStation` class shows the observer pattern in use.

{% highlight java %}
public class WeatherStation {

	public static void main(String[] args) {
		WeatherData weatherData = new WeatherData();

		CurrentConditionsDisplay currentDisplay =
			new CurrentConditionsDisplay(weatherData);
		StatisticsDisplay statisticsDisplay = new StatisticsDisplay(weatherData);

		weatherData.setMeasurements(80, 65, 30.4f);
		weatherData.setMeasurements(82, 70, 29.2f);
	}
}
{% endhighlight %}

Starting this application results in the following output.

{% highlight bash %}
$java WeatherStation
Current conditions: 80.0F degrees and 65.0% humidity
Avg/Max/Min temperature = 80.0/80.0/80.0
Current conditions: 82.0F degrees and 70.0% humidity
Avg/Max/Min temperature = 81.0/82.0/80.0
{% endhighlight %}
