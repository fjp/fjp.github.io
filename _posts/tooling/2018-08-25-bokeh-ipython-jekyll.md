---
layout: single
title:  "Interactive Bokeh for Jekyll using iPython"
date:   2018-08-25 14:24:31 +0100
categories: [tooling, python]
tags: [ipython, python, bokeh, jekyll, tooling, html, include, files]
use_math: true
classes: wide
toc: true
# toc_label: "Unscented Kalman Filter"
image:
  feature: /assets/posts/2017-12-03-dynamic-models/lat-lon-forces.png
  thumb: /assets/posts/2017-12-03-dynamic-models/lat-lon-forces.png #keep it square 200x200 px is good
---


## Extend Jekyll to include html files

Create new file `notebook` inside the `_include` folder of a standard jekyll environment with the following content.

{% highlight html %}
<script>
  function resizeIframe(obj) {
    obj.style.height = obj.contentWindow.document.body.scrollHeight + 'px';
  }
</script>

<iframe
    src="{{ include.path }}" class="iframe" scrolling="no" frameborder="0"
    onload="resizeIframe(this)" width="100%">
</iframe>
{% endhighlight %}

Now to include a html file inside a post use the following command:

{% highlight html %}
{% raw %}
{% include notebook path="/path/to/html_file.html" %}
{% endraw %}
{% endhighlight %}


In my case I put all the html files into subfolders in the root folder `asset`.  

The results where I included html files create with bokeh and jupyter notebook looks like this:

{% include notebook path="/assets/notebooks/color_scatter.html" %}

Above you can see a bokeh plot that uses no interactive functionality. However,
below is an example wit the possibility to interact by moving the slider.




## Bokeh and Jupyter Notebook

To create the html files above I used jupyter notebook. Below you see the python code example I got from [here](https://bokeh.pydata.org/en/latest/docs/user_guide/interaction/callbacks.html#userguide-interaction-jscallbacks).

{% highlight python %}
from bokeh.layouts import column
from bokeh.models import CustomJS, ColumnDataSource, Slider
from bokeh.plotting import Figure, output_file, show

output_file("callback.html")

x = [x*0.005 for x in range(0, 200)]
y = x

source = ColumnDataSource(data=dict(x=x, y=y))

plot = Figure(plot_width=400, plot_height=400)
plot.line('x', 'y', source=source, line_width=3, line_alpha=0.6)

def callback(source=source, window=None):
    data = source.data
    f = cb_obj.value
    x, y = data['x'], data['y']
    for i in range(len(x)):
        y[i] = window.Math.pow(x[i], f)
    source.change.emit()

slider = Slider(start=0.1, end=4, value=1, step=.1, title="power",
                callback=CustomJS.from_py_func(callback))

layout = column(slider, plot)

show(layout)
{% endhighlight %}


After executing the cell I got an error message which stated that I need to install bokeh flexx using the followign command:


{% highlight bash %}
conda install -c bokeh flexx
Fetching package metadata ...........


PackageNotFoundError: Package not found: '' Package missing in current osx-64 channels:
  - flexx
{% endhighlight %}

After executing this command on mac os I got the missing package error above.
On the Bokeh [installation page](https://bokeh.pydata.org/en/latest/docs/dev_guide/setup.html#conda) for conda, I found that I have to add the following channels.

{% highlight bash %}
conda config --append channels bokeh
conda config --append channels conda-forge
{% endhighlight %}

## Ressources

- More Bokeh examples can be found [here](http://bokeh.pydata.org/en/0.11.1/docs/user_guide/interaction.html).
