---
layout: single #collection
title: ROS rqt plugin for turtlesim
permalink: /ros/rqt-turtle/
excerpt: "Plugin for ROS rqt to draw in turtlesim using turtlebot."
date: 2020-08-23 20:00:35 +0200
categories: [robotics]
tags: [ros, rqt, noetic, plugin, qt5, qt, turtlesim, turtlebot]
comments: true
use_math: true
toc: true
classes: wide
header:
  teaser: /assets/ros/rqt_turtle/turtle_plugin_ui.png
  overlay_image: https://raw.githubusercontent.com/fjp/rqt-turtle/master/docs/rqt_turtle-draw-image.gif
  overlay_filter: 0.5 # same as adding an opacity of 0.5 to a black background
  caption: rqt_turtle
  show_overlay_excerpt: true
sidebar:
  nav: ros
---

This project is part of the Robocademy 
[Robot Operating System Learning Path](https://robocademy.com/2020/06/25/enroll-in-robot-operating-system-learning-path-by-lentin-joseph/) by 
[Lentin Joseph](https://lentinjoseph.com/). 
The project shows how the `rqt_turtle` plugin for ROS' [GUI framework rqt](http://wiki.ros.org/rqt) was created.

The final plugin looks like this:

<figure>
    <a href="https://raw.githubusercontent.com/fjp/rqt-turtle/master/docs/images/rqt-turtle.png"><img src="https://raw.githubusercontent.com/fjp/rqt-turtle/master/docs/images/rqt-turtle.png"></a>
    <figcaption>Loaded rqt turtle plugin.</figcaption>
</figure>

The video below gives more insights on what is currently implemented:

[![rqt turtle YouTube](http://img.youtube.com/vi/2IQtxEmP2a4/0.jpg)](https://youtu.be/2IQtxEmP2a4)

The plugin can be used to draw inside [turtlesim](http://wiki.ros.org/turtlesim) with turtlebot.
Although the following description might help you to write your own rqt plugin, also have a look at the official [rqt tutorials](http://wiki.ros.org/rqt/Tutorials). 
There are tutorials explaining how to write plugins using python or C++. The `rqt_turtle` plugin is written in C++.
Note that the following documentation doesn't list the full source code. For this The [source code](https://github.com/fjp/rqt-turtle) is hosted on GitHub.

The code tries to follow the [ROS CppStyleGuide](http://wiki.ros.org/CppStyleGuide) and the code API documentation can be created with [doxygen](http://wiki.ros.org/Doxygen). The code was written with the [vscode ROS plugin](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros)
that allows to debug rqt plugins using attach to process method. 
See the [documentation](https://github.com/ms-iot/vscode-ros/blob/master/doc/debug-support.md) on how to setup debugging and its debugging capabilities.

## C++ vs Python

Note that most rqt plugins are wirtten in Python and it is even recommended to write them in Python. 
However, because of few documented C++ plugins, this project shows how to write a plugin in C++ and it is similar to the
[`rqt_image_view` plugin](http://wiki.ros.org/rqt_image_view) that is also programmed in C++. 
In case you are planning to reuse existing rqt plugins make sure to use the 
language that they are written in. As you will see later in this writeup, I had to use some compromise to use existing rqt plugins which are were written in Python. Another drawback is that the [XML-RPC](https://en.wikipedia.org/wiki/XML-RPC) functionality available from the ROS Master API is not well documented. 
For example using Python to obtain service or topic information such as types or arguments is much easier with Python.


## Create the Empty ROS Package

The first step is to create an empty ROS package and specify the required dependencies. 
Note, that it is possible to add missing dependencies later on.
Inside a [ros workspace](http://wiki.ros.org/catkin/workspaces) use the [`catkin create`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_create.html) command from [catkin-tools](https://catkin-tools.readthedocs.io/en/latest/) to creat the empty `rqt_turtle` package:

```console
$ catkin create pkg rqt_turtle \
    -a "Franz Pucher" "ros@fjp.at" \
    -m "Franz Pucher" "ros@fjp.at" \
    -l "MIT" \
    -d "rqt plugin for ROS rqt to draw in turtlesim using turtlebot." \
    --catkin-deps roscpp rqt_gui rqt_gui_cpp
```

## Modify the package content

This section follows the instructions in [wiki.ros.org/rqt/Tutorials/Create your new rqt plugin](http://wiki.ros.org/rqt/Tutorials/Create%20your%20new%20rqt%20plugin).

### package.xml

To make the rqt plugin discoverable, for catkin, we must declare the plugin in the `package.xml` by referencing a `plugin.xml`.

```xml
  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->
    <rqt_gui plugin="${prefix}/plugin.xml"/>
  </export>
```

For more information see the [catkin library dependencies](http://docs.ros.org/indigo/api/catkin/html/howto/format2/catkin_library_dependencies.html).

### plugin.xml

Now create the `plugin.xml` with the following content:

```xml
<!-- To make the plugin discoverable by rqt the path to the library librqt_turtle.so needs to be specified -->
<!-- The installed plugin library can be found in the devel/lib/ folder. -->
<library path="lib/librqt_turtle">
  <class name="rqt_turtle/TurtlePlugin" type="rqt_turtle::TurtlePlugin" base_class_type="rqt_gui_cpp::Plugin">
    <description>
      Plugin for ROS rqt to draw in turtlesim using turtlebot.
    </description>
    <qtgui>
      <!-- group the plugin into the folder Robot Tools... -->
      <group>
        <label>Robot Tools</label>
        <icon type="theme">folder</icon>
        <statustip>Plugins related to robot tools.</statustip>
      </group>
      <label>TurtleSim</label>
      <icon type="theme">input-tablet</icon>
      <statustip>Plugin for ROS rqt to draw in turtlesim using turtlebot.</statustip>
    </qtgui>
  </class>
</library>
```

A description of these xml attributes is found in the [rqt tutorial](http://wiki.ros.org/rqt/Tutorials/Create%20your%20new%20rqt%20plugin#Attributes_of_library_element_in_plugin.xml).

The first attribute `library path` specifies the path to the library `librqt_turtle.so` which makes the plugin discoverable for rqt.
In this case the installed plugin library can be found in the ros workspace inside the `devel/lib/` folder.

The `rqt_turtle` plugin will be grouped into the existing Robot Tools menu of `rqt` with the icon type `input-table`. The names
for other icons can be found in the [Icon Naming Specification](https://specifications.freedesktop.org/icon-naming-spec/icon-naming-spec-latest.html).


## Design the UI

ROS rqt plugins are based on the [Qt framework](https://www.qt.io/product/framework). When [installing Qt](https://wiki.qt.io/Install_Qt_5_on_Ubuntu), 
using the official Ubuntu package `sudo apt-get install qt5-default` the required tools to develop Qt applications are installed too.
Qt provides its own Qt Creator IDE and a Designer that lets you create GUIs that work are cross platform, meaning they will work on Linux, macOS and Windows.


To design an ui file open a terminal and enter `designer`

<figure>
    <a href="/assets/ros/qt/designer.png"><img src="/assets/ros/qt/designer.png"></a>
    <figcaption>Qt designer - New Form.</figcaption>
</figure>

Select `Widget` from the New Form Wizard Dialog because `rqt` can be composed with plugins that are of type Qt Widgets.
This means you can turn existing Qt Widgets into `rqt` plugins.

The following image shows the design of the `rqt_turtle` plugin. On the right you can see the components. 

<figure>
    <a href="/assets/ros/rqt_turtle/turtle_plugin_ui.png"><img src="/assets/ros/rqt_turtle/turtle_plugin_ui.png"></a>
    <figcaption>Qt Designer - rqt turtle plugin.</figcaption>
</figure>

## Write the Plugin Code


### ROS Components

```cpp
// Deprecated
// See: http://wiki.ros.org/pluginlib#pluginlib.2Fpluginlib_groovy.Simplified_Export_Macro
//PLUGINLIB_DECLARE_CLASS(rqt_turtle, TurtlePlugin, rqt_turtle::TurtlePlugin, rqt_gui_cpp::Plugin)
PLUGINLIB_EXPORT_CLASS(rqt_turtle::TurtlePlugin, rqt_gui_cpp::Plugin)
```

To obtain a list of existing turtles it would be possible to execute `rostopic list` from the command line and filter for the
topics that include `/turtle_name/pose` or `/turtle_name/cmd_vel`. The same can be achieved within a [`roscpp`](http://docs.ros.org/noetic/api/roscpp/html/) 
node using the [`bool ros::master::getTopics(V_TopicInfo& topics)`](http://docs.ros.org/noetic/api/roscpp/html/namespaceros_1_1master.html#aa922f42cf983b06edb4cf3de7d7ce211) method, see also [this example](https://stackoverflow.com/questions/26785675/ros-get-current-available-topic-in-code-not-command):

```cpp
void TurtlePlugin::updateTurtleTree()
{
    // https://stackoverflow.com/questions/26785675/ros-get-current-available-topic-in-code-not-command
    // Use XML-RPC ROS Master API to get the topic names
    // Then filter for topics containing pose (which belongs to a turtle)
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    ros::NodeHandle nh = getNodeHandle();
    for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++)
    {
        const ros::master::TopicInfo& info = *it;
        ROS_INFO_STREAM("topic_" << it - master_topics.begin() << ": " << info.name);
        QString topic_name = QString::fromStdString(info.name);
        if (topic_name.contains(QString("/pose")))
        {
            QStringList topic_name_parts = topic_name.split(QRegExp("\\/"), QString::SkipEmptyParts);
            std::string turtle_name = topic_name_parts[0].toStdString();
            ROS_INFO("topic_name_part 0: %s", turtle_name.c_str());

            // Wait for a single pose message to arrive on the turtlesim::Pose topic
            turtlesim::PoseConstPtr pose = ros::topic::waitForMessage<turtlesim::Pose>(topic_name.toStdString());
            ROS_INFO("Pose received: x: %f, y: %f, theta: %f", pose->x, pose->y, pose->theta);

            // Create new turtle in turtle vector
            // Note: assume that the pen is toggled on
            QSharedPointer<Turtle> turtle = QSharedPointer<Turtle>(new Turtle(turtle_name, *pose));
            turtles_[QString::fromStdString(turtle_name)] = turtle;
        }
    }

    // Insert the turtles into the QTreeWidget
    for (auto turtle : turtles_)
    {
        m_pUi->treeTurtles->insertTopLevelItem(0, turtle->toTreeItem(m_pUi->treeTurtles));
    }
}
```

The `TurtlePlugin::updateTurtleTree()` method can be called to get a list of currently active turtles in turtlesim. 
This means that turtles can also be added the the `rosservice` command in another terminal and after refreshing the plugin the newly spawned turtles 
will be listed.


### Qt Components

After designing UIs there are (at least) three possible ways to use them inside an application, which are explained in more detail in the [documentation](https://doc.qt.io/qt-5/designer-using-a-ui-file.html):

- [Direct Approach](https://doc.qt.io/qt-5/designer-using-a-ui-file.html#the-direct-approach)
- [Single Inheritance](https://doc.qt.io/qt-5/designer-using-a-ui-file.html#the-single-inheritance-approach)
  - [Using a Member Variable](https://doc.qt.io/qt-5/designer-using-a-ui-file.html#using-a-member-variable)
  - [Using a Pointer Member Variable](https://doc.qt.io/qt-5/designer-using-a-ui-file.html#using-a-pointer-member-variable) **recommended approach***
- [Multiple Inheritance Approach](https://doc.qt.io/qt-5/designer-using-a-ui-file.html#the-multiple-inheritance-approach)


All of the approaches would work for `rqt_turtle` except the `Direct Approach` because it doesn't allow us to use custom slots.
The reason to choose the `Single Inheritance using a Pointer Member Variable` approach is to avoid including the generated `ui_name_of_gui.h` inside the applications header file. Instead a forwrad declaradtion is made which allows to use the ui as pointer member. Then the generated `ui_name_of_gui.h`
can only be included in the applications source (`cpp`) file. The `Single Inheritance using a Pointer Member Variable` approach is used for all
ui files that are used in the `rqt_turtle` plugin.


### Signal and Slots

The Qt framework uses a [signal and slot mechanism](https://doc.qt.io/qt-5/signalsandslots.html) to allow the communication between objects.
For the `rqt_turtle` plugin it is used to connect for example the buttons with the plugin's main class, called `TurtlePlugin`, which handles
the button presses in its custom slots.

All the available turtles inside turtlesim are displayd in the `treeTurtle` widget of type [`QTreeWidget`](https://doc.qt.io/qt-5/qtreewidget.html).
For this the previously mentioned `TurtlePlugin::updateTurtleTree()` is used. One important feature of the `treeTurtle` is to obtain the selected turtles.
This is done by connecting the `QTreeWidget::itemSelectionChanged()` signal of `treeTurtle`

```cpp
connect(ui_->treeTurtles, SIGNAL(itemSelectionChanged()), 
                this, SLOT(on_selection_changed()));
```

with the custom `TurtlePlugin::on_selection_chagned()` slot:

```cpp
void TurtlePlugin::on_selection_changed()
{
    // Get list of selected turtles
    auto selected_items = m_pUi->treeTurtles->selectedItems();
    selected_turtles_.clear();
    if (selected_items.empty())
    {
        return;
    }
    QString turtle_name;
    std::stringstream ss;
    for (auto item : selected_items)
    {
        turtle_name = item->text(0);
        selected_turtles_.push_back(turtle_name);
        ss << str(turtle_name) << " ";
    }
    ROS_INFO("Selected %s", ss.str().c_str());
}
```

This will store the currently selected turtle names as [`QString`](https://doc.qt.io/qt-5/qstring.html)s in the member variable `selected_turtles_` of type `QVector<QString>`.
If no turtles are selected the vector will be empty.

To store the turtles with their settings the plugin uses [`QMap`](https://doc.qt.io/qt-5/qmap.html)

```cpp
/// Vector to keep track of all turtles (keep turtles on the heap using vector of shared pointers)
QMap<QString, QSharedPointer<Turtle> > turtles_;
```

This maps the `QString` turtle names, for example obtained from `selected_turtles_`, to `Turtle` object pointers of type [`QSharedPointer<Turtle>`](https://doc.qt.io/qt-5/qsharedpointer.html). The `Turtle` class is used to encapsulate a turtle's [`turtlesim::Pose`](http://docs.ros.org/noetic/api/turtlesim/html/msg/Pose.html), [`turtlesim::SetPen`](http://docs.ros.org/noetic/api/turtlesim/html/srv/SetPen.html) and name.
The `QSharedPointer` take care to delete turtles when no references are pointing to them.


There exist also `QTreeView` or `QListView` elements inside Qt but those don't provide as much functionality than their `*Widget` counterparts do.
If you plan the implement such elements using your own functionality then probably the right choice is to use the rudimentary `*View` elements.
Otherwise the `*Widget` will be the right choice.
{: .notice }

The main `rqt_turtle` ui provides the following [QPushButton](https://doc.qt.io/qt-5/qpushbutton.html)s, which have their [`clicked()` signal](https://doc.qt.io/qt-5/qabstractbutton.html#clicked) connected to corresponding slots that do the acutal work:

- Spawn: connected to the `void TurtlePlugin::on_btnSpawn_clicked()` slot to open a new ui `ServiceCaller.ui` dialog.
- Reset: connected to the `void TurtlePlugin::on_btnReset_clicked()` which calls the [`reset` service](http://wiki.ros.org/turtlesim#Services) 
of turtlesim to bring it to the start configuration and set the background color to the value of the currently set background parameters.
- Color: connected to the `on_btnColor_clicked` slot which sets the RGB color values on the [ROS parameter server](http://wiki.ros.org/Parameter%20Server) 
used by turtlesim as background color, see [turtlesim parameters](http://wiki.ros.org/turtlesim#Parameters). 
After changing the color, the [`clear` service](http://wiki.ros.org/turtlesim#Services) is called to immediately update the new color.

The `Spawn` button and the two `Teleport Abs` and `Teleport Rel` buttons open a new dialog window. 
Currently there are three different dialog implementations:

- `ServiceCaller.ui` for "Spawn", "Teleport Abs" and "Teleport Rel" buttons.
- `Draw.ui` for the Draw button.
- `Task.ui` called within the Draw dialog to show the drawing progress.

For each ui there exists a class that inherits from [`QDialog`](https://doc.qt.io/qt-5/qdialog.html) - 
following the same `Single Inheritance using a Pointer Member Variable` approach described above.
This allows us to show a modal dialog with the [`int QDialog::exec()`](https://doc.qt.io/qt-5/qdialog.html#exec) method, 
that can be blocked until the user closes it. Depending on what the user pressed in the dialog it's possible to call 
[`void QDialog::accept()`](https://doc.qt.io/qt-5/qdialog.html#accept) or  [`void QDialog::reject()`](https://doc.qt.io/qt-5/qdialog.html#reject) 
slots which will hide the dialog and set the return value of the `QDialog::exec()` method. 
Note, that instead of `QDialog::exec()` [`QDialog::open()`](https://doc.qt.io/qt-5/qdialog.html#open) 
is recommended to be used in combination with the [`void QDialog::finished(int result)`](https://doc.qt.io/qt-5/qdialog.html#finished) signal.

The following two sections will describe `ServiceCaller`, `Draw` and `Task` uis and their corresponding classes in more detail.

One way to get data from a dialog is to implement a member function to obtain it.
[Example1](https://stackoverflow.com/questions/3585774/how-to-pass-data-from-a-qdialog), [Example2](https://www.qtcentre.org/threads/25690-How-to-pass-data-from-QDialog-to-MainWindow). For this plugin this method is required to obtain the service message data entered by the user.
{: .notice }

### Service Caller

The `ServiceCaller` class together with it's ui is used to call the available [services of turtlesim](http://wiki.ros.org/turtlesim#Services).

ROS provides the `rosservice` tool which has the `list` subcommand to list all the available services that are 
currently registered with the ROS master.


Note that it would be easier to use `rospy` to obtain service infos from the master.
Using `roscpp` makes it harder to get the required information.
{: .notice }

The following list shows three approaches to get information from the ROS master when using `roscpp`:

- [rosapi](http://wiki.ros.org/rosapi) not covered here because it would require adding it as another dependency (reference [answer](https://answers.ros.org/question/152481/get-service-type-from-c/), [example](https://answers.ros.org/question/108176/how-to-list-all-topicsservices-that-are-known-by-the-server-with-roscpp/)).
- [XML-RPC](https://en.wikipedia.org/wiki/XML-RPC) calls using [ROS Master API](http://wiki.ros.org/ROS/Master_API) This would be the way to go if the plugin was written in Python. The ROS Master API seems to be incomplete for C++ (reference [answer](https://answers.ros.org/question/151611/rosservice-list-and-info-from-c/)).
- [Calling terminal commands from C++](https://stackoverflow.com/questions/478898/how-do-i-execute-a-command-and-get-the-output-of-the-command-within-c-using-po) A hacky solution to execute ros commands such as `rosservice list` within C++. The details of this approach are explained in the next section.

Before that, it is worth mentioning how the service calls are implemented:

For example the `Spawn`, `Teleport Abs` and `Teleport Rel` service calls make use of the [`ros::service::calls()`] method.
After pressing one of the buttons a new `ServiceCaller` class is created with the `TurtlePlugin::widget_` member as parent and the desired service name.

```cpp
std::string service_name = "/spawn";
service_caller_dialog_ = new ServiceCaller(widget_, service_name);
```

The name is used to fill the `ServiceCaller::ui_::request_tree_widget`. For this the mentioned `rosservice args` command is called from within 
the `ServiceCaller::createTreeItems(std::string service_name)` method:

```cpp
void ServiceCaller::createTreeItems(std::string service_name)
{
    ROS_INFO("Create Tree Items for service %s", service_name.c_str());

    // Get service type and args
    std::string cmd_type = "rosservice type " + service_name;
    ROS_INFO("cmd_type: %s", cmd_type.c_str());
    std::string service_type = exec_cmd(cmd_type);
    std::string cmd_args = "rosservice args " + service_name;
    std::string service_args = exec_cmd(cmd_args);
    QString qstr_service_args_line = QString::fromStdString(service_args);
    ROS_INFO("Service args: %s", qstr_service_args_line.toStdString().c_str());
    QStringList qstr_args = qstr_service_args_line.split(QRegExp("\\s+"), QString::SkipEmptyParts);

    // https://doc.qt.io/qt-5/qtreewidget.html#details
    QList<QTreeWidgetItem *> items;
    for (int i = 0; i < qstr_args.size(); ++i)
    {
        QStringList i_args;
        i_args.append(qstr_args[i]);
        // TODO type
        i_args.append(QString::fromStdString("float32"));
        // TODO init value depending on type
        i_args.append(QString::fromStdString("0.0"));
        QTreeWidgetItem* item = new QTreeWidgetItem(static_cast<QTreeWidget *>(nullptr), i_args);
        item->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        items.append(item);
    }
    m_pUi->request_tree_widget->insertTopLevelItems(0, items);
}
```

Here, `request_tree_widget` is filled with the service arguments, see the services `Spawn`, `TeleportAbsolute` and `TeleportRelative` as an example in the following figures:


<figure class="third">
  <a href="https://raw.githubusercontent.com/fjp/rqt-turtle/master/docs/images/service-caller-spawn.png"><img src="https://raw.githubusercontent.com/fjp/rqt-turtle/master/docs/images/service-caller-spawn.png"></a>
  <a href="https://raw.githubusercontent.com/fjp/rqt-turtle/master/docs/images/service-caller-teleport-abs.png"><img src="https://raw.githubusercontent.com/fjp/rqt-turtle/master/docs/images/service-caller-teleport-abs.png"></a>
  <a href="https://raw.githubusercontent.com/fjp/rqt-turtle/master/docs/images/service-caller-teleport-rel.png"><img src="https://raw.githubusercontent.com/fjp/rqt-turtle/master/docs/images/service-caller-teleport-rel.png"></a>
  <figcaption>ServiceCaller showing fields of the Spawn service.</figcaption>
</figure>

In the current implementation the types are not checked and therefore not displayed correctly, see future work section below.
{: .notice }

After the user enters the desired pose and name of the turtle (when the `spawn` service was used) the data can be obtained in the `TurtlePlugin` class
with the `QVariantMap ServiceCaller::getRequest()` method:

```cpp
QVariantMap ServiceCaller::getRequest()
{
    QVariantMap map;

    // https://doc.qt.io/archives/qt-4.8/qtreewidgetitemiterator.html#details
    QTreeWidgetItemIterator it(m_pUi->request_tree_widget);
    while (*it)
    {
        QString variable = (*it)->text(0);
        map[variable] = (*it)->text(2);
        ++it;
    }
    return map;
}
```

It returns a [`QVariantMap`](https://doc.qt.io/archives/qt-5.5/qvariant.html#QVariantMap-typedef) that is a `QMap<QString, QVariant>`.
Using [`QVariant`](https://doc.qt.io/archives/qt-5.5/qvariant.html) as value is useful to deal with different types of data, for example `string` and `float`. 
With this, the caller (in this case `TurtlePlugin`) knows what to expect from the `ServiceCaller` class and the `ServiceCaller` can be reused 
for different types of [services provided by turtlesim](http://wiki.ros.org/turtlesim#Services).

The ServiceCaller currently cannot handle nested service messages. For this use the [rqt service caller](http://wiki.ros.org/rqt_service_caller) plugin.
{: .notice }


#### Command Line Interface Approach

This approach, to get for example the service list is kind of a hack and doesn't leverage the [XML-RPC](https://en.wikipedia.org/wiki/XML-RPC) backend of ROS.
However, it is the simpler approach, which is why it's used for the first/current version of this plugin.

The following C++ snippet shows how to get the output of a terminal command ([source](https://stackoverflow.com/questions/478898/how-do-i-execute-a-command-and-get-the-output-of-the-command-within-c-using-po)). In this plugin it's used to get the output of ros commands within a `roscpp` node.

```cpp
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>

std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}
```


#### XML-RPC Approach

This approach is currently not implemented. Instead the CLI approach described above is used to call ros commands within a roscpp node.
{: .notice }

To get the service list in our C++ code we can make use
of XML-RPC which ROS uses under the hood for its communication. With it we can query the 
ROS master through the [ROS Master API](http://wiki.ros.org/ROS/Master_API).

First we need to include the `master.h`:

```cpp
#include <ros/master.h>
```

Then we can create `XmlRpc::XmlRpcValue` request, response and payload objects which are required for the call.

```cpp
// The request value can be set to any value
XmlRpc::XmlRpcValue request = "/node"; 
// The response object will receive the server messsage in xml format
XmlRpc::XmlRpcValue response;
XmlRpc::XmlRpcValue payload;
```

Finally, we need to call the `getSystemState` method of the master using the [`ros::master::execute`](http://docs.ros.org/noetic/api/roscpp/html/namespaceros_1_1master.html#a6148ee923ef1602d2093daff82573043) method.

```cpp
// http://wiki.ros.org/ROS/Master_API
// The following calls the getSystemState method with the previously defined request
// and returns the response and the payload.
ros::master::execute("getSystemState", request, response, payload, true);
```

To view the xml response we can make use of `XmlRpc::XmlRpcValue::toXml`:

```cpp
ROS_INFO("%s", response.toXml().c_str());
```

The result will be an array that holds the xml tree:

```
<value><array><data><value><i4>1</i4></value><value>current system state</value><value><array><data><value><array><data><value><array><data><value>/rosout_agg</value><value><array><data><value>/rosout</value></data></array></value></data></array></value><value><array><data><value>/rosout</value><value><array><data><value>/rqt_gui_cpp_node_43389</value><value>/rqt_gui_cpp_node_43761</value><value>/rqt_gui_cpp_node_45610</value><value>/rqt_gui_cpp_node_47424</value><value>/rqt_gui_cpp_node_50133</value><value>/rqt_gui_cpp_node_50310</value><value>/rqt_gui_cpp_node_50562</value><value>/rqt_gui_cpp_node_57695</value></data></array></value></data></array></value></data></array></value><value><array><data><value><array><data><value>/rosout</value><value><array><data><value>/rosout</value></data></array></value></data></array></value></data></array></value><value><array><data><value><array><data><value>/rosout/get_loggers</value><value><array><data><value>/rosout</value></data></array></value></data></array></value><value><array><data><value>/rosout/set_logger_level</value><value><array><data><value>/rosout</value></data></array></value></data></array></value><value><array><data><value>/rqt_gui_cpp_node_43389/get_loggers</value><value><array><data><value>/rqt_gui_cpp_node_43389</value></data></array></value></data></array></value><value><array><data><value>/rqt_gui_cpp_node_43389/set_logger_level</value><value><array><data><value>/rqt_gui_cpp_node_43389</value></data></array></value></data></array></value><value><array><data><value>/rqt_gui_cpp_node_43761/get_loggers</value><value><array><data><value>/rqt_gui_cpp_node_43761</value></data></array></value></data></array></value><value><array><data><value>/rqt_gui_cpp_node_43761/set_logger_level</value><value><array><data><value>/rqt_gui_cpp_node_43761</value></data></array></value></data></array></value><value><array><data><value>/rqt_gui_cpp_node_45610/get_loggers</value><value><array><data><value>/rqt_gui_cpp_node_45610</value></data></array></value></data></array></value><value><array><data><value>/rqt_gui_cpp_node_45610/set_logger_level</value><value><array><data><value>/rqt_gui_cpp_node_45610</value></data></array></value></data></array></value><value><array><data><value>/rqt_gui_cpp_node_47424/get_loggers</value><value><array><data><value>/rqt_gui_cpp_node_47424</value></data></array></value></data></array></value><value><array><data><value>/rqt_gui_cpp_node_47424/set_logger_level</value><value><array><data><value>/rqt_gui_cpp_node_47424</value></data></array></value></data></array></value><value><array><data><value>/rqt_gui_cpp_node_50133/get_loggers</value><value><array><data><value>/rqt_gui_cpp_node_50133</value></data></array></value></data></array></value><value><array><data><value>/rqt_gui_cpp_node_50133/set_logger_level</value><value><array><data><value>/rqt_gui_cpp_node_50133</value></data></array></value></data></array></value><value><array><data><value>/rqt_gui_cpp_node_50310/get_loggers</value><value><array><data><value>/rqt_gui_cpp_node_50310</value></data></array></value></data></array></value><value><array><data><value>/rqt_gui_cpp_node_50310/set_logger_level</value><value><array><data><value>/rqt_gui_cpp_node_50310</value></data></array></value></data></array></value><value><array><data><value>/rqt_gui_cpp_node_50562/get_loggers</value><value><array><data><value>/rqt_gui_cpp_node_50562</value></data></array></value></data></array></value><value><array><data><value>/rqt_gui_cpp_node_50562/set_logger_level</value><value><array><data><value>/rqt_gui_cpp_node_50562</value></data></array></value></data></array></value><value><array><data><value>/rqt_gui_cpp_node_57695/get_loggers</value><value><array><data><value>/rqt_gui_cpp_node_57695</value></data></array></value></data></array></value><value><array><data><value>/rqt_gui_cpp_node_57695/set_logger_level</value><value><array><data><value>/rqt_gui_cpp_node_57695</value></data></array></value></data></array></value></data></array></value></data></array></value></data></array></value>
```

To access the relevant part - the service list - we can use the following for loop:

```cpp
const int num_services = response[2][2].size()

// Create a string array holding the service names
std::string services[num_services];
for(int x=0; x < num_services; x++)
{
    // Get the xml part containing a service value
    // e.g., <value>/rosout/get_loggers</value>
    std::string service_name = response[2][2][x][0].toXml().c_str();
    
    // remove <value> and </value> from the string
    service_name.erase(service_name.begin(), service_name.begin()+7);
    service_name.erase(service_name.end()-8, service_name.end());
    services[x] = service_name;
    ROS_INFO(service_name.c_str());
}
```

<details markdown="1"><summary>Expand for an alternative solution.</summary>

An alternative to the above snippet would be to use [`TinyXML`](http://www.grinninglizard.com/tinyxmldocs/index.html), a C++ XML parsing library.
To load the xml from the response string use `TiXmlDocument::Parse()` (see this [SO answer](https://stackoverflow.com/questions/2862892/can-tinyxml-load-xml-from-string-instead-of-file)):

```cpp
TiXmlDocument doc;
doc.Parse((const char*)response.toXml().c_str(), 0, TIXML_ENCODING_UTF8);
```

Then it should be possible to parse the XML with the steps outlined in the [TinyXML tutorial](http://www.grinninglizard.com/tinyxmldocs/tutorial0.html).

```cpp
std::string message;
MessageMap messages;

TiXmlHandle hDoc(&doc);
TiXmlElement* pElem;
TiXmlHandle hRoot(0);

// block: name
{
    pElem=hDoc.FirstChildElement().Element();
    // should always have a valid root but handle gracefully if it does
    if (!pElem) return;
    m_name=pElem->Value();

    // save this for later
    hRoot=TiXmlHandle(pElem);
}

// block: string table
{
    messages.clear(); // trash existing table

    pElem=hRoot.FirstChild( "Messages" ).FirstChild().Element();
    for( pElem; pElem; pElem=pElem->NextSiblingElement())
    {
        const char *pKey=pElem->Value();
        const char *pText=pElem->GetText();
        if (pKey && pText) 
        {
            messages[pKey]=pText;
        }
    }
}
```
</details>


Useful references for working with XML-RPC in the roscpp client library are this [answer](https://answers.ros.org/question/151611/rosservice-list-and-info-from-c/?answer=152421#post-id-152421) and the [ROS Master API Wiki page](http://wiki.ros.org/ROS/Master_API).


### Draw Dialog

The `Draw.ui` declares a `QWidget` named `DrawWidget` that is used as a `QDialog`. 
It provides two tabs, one to draw a shape using the `turtle_shape` service from the [`turtle_actionlib`](http://wiki.ros.org/turtle_actionlib).


<details markdown="1"><summary>Expand for rqt turtle draw shape demo.</summary>

![rqt_turtle-draw-shape](https://raw.githubusercontent.com/fjp/rqt-turtle/master/docs/rqt_turtle-draw-shape-cancel.gif)

</details>

And the second tab to let one or more turtles draw an image.

<details markdown="1"><summary>Expand for rqt turtle draw image animation.</summary>

![rqt_turtle-draw-image](https://raw.githubusercontent.com/fjp/rqt-turtle/master/docs/rqt_turtle-draw-image-multi.gif)

</details>

Both of the following guis make use of [`QRunnable`](https://doc.qt.io/qt-5/qrunnable.html) and [`QThreadPool`](https://doc.qt.io/qt-5/qthreadpool.html) to avoid blocking the gui and allow to cancel the ongoing drawing task.
The implementation of both workers [`ActionWorker`](https://github.com/fjp/rqt-turtle/blob/master/rqt_turtle/src/rqt_turtle/action_worker.cpp) 
and [`ImageWorker`](https://github.com/fjp/rqt-turtle/blob/master/rqt_turtle/src/rqt_turtle/image_worker.cpp) uses concepts from the [PyQt5 Book](https://www.learnpyqt.com/pyqt5-book/).

#### Draw Shape

The [`turtle_actionlib`](http://wiki.ros.org/turtle_actionlib) provides a `shape_server` that is used to let the turtle named `turtle1` draw
a shape. For this the `Draw` class defines an action client `actionlib::SimpleActionClient<turtle_actionlib::ShapeAction> ac_;`
which is used to send an action goal containing the number of edges and the radius of the shape to the server. 
See also the [detailed description](http://wiki.ros.org/actionlib/DetailedDescription) of the [`actionlib`](http://wiki.ros.org/actionlib?distro=noetic) to learn more about how it works.
The following shows the gui to adjust these two settings including the timeout to automatically cancel the goal:

<figure>
    <a href="https://raw.githubusercontent.com/fjp/rqt-turtle/master/docs/images/draw-shape.png"><img src="https://raw.githubusercontent.com/fjp/rqt-turtle/master/docs/images/draw-shape.png"></a>
    <figcaption>Draw Shape Dialog.</figcaption>
</figure>


With the [`turtle_actionlib/Shape.action`](http://docs.ros.org/noetic/api/turtle_actionlib/html/action/Shape.html) its possible to specify the number of edges (`int32`) and the radius (`float32`). 
To send the specified shape (also known as goal) the client is used as follows:

```cpp
void Draw::drawShape()
{
    ROS_INFO("Waiting for action server to start.");

    if (!ac_.isServerConnected())
    {
        QMessageBox msgBox;
        msgBox.setText("Action server not connected");
        msgBox.setInformativeText("Please run 'rosrun turtle_actionlib shape_server' and press Ok or cancel \
                                   to avoid blocking rqt_turtle gui while wating for shape_server.");
        msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
        int ret = msgBox.exec();
        if (ret == QMessageBox::Cancel)
        {
            return;
        }
    }

    /// Wait for the action server to start
    ac_.waitForServer(); //will wait for infinite time
    ROS_INFO("Action server started, sending goal.");

    /// Create ActionWorker which will send a goal to the action server
    int edges = ui_->lineEditEdges->text().toInt();
    float radius = ui_->lineEditRadius->text().toFloat();
    float timeout = ui_->lineEditTimeout->text().toFloat();
    ActionWorker* action_worker = new ActionWorker(ac_, edges, radius, timeout);

    ui_->btnDraw->setText(QString("Cancel Goal"));
    disconnect(ui_->btnDraw, SIGNAL(clicked()), this, SLOT(on_btnDraw_clicked()));
    connect(ui_->btnDraw, SIGNAL(clicked()), action_worker, SLOT(kill()));
    connect(ui_->btnDraw, SIGNAL(clicked()), this, SLOT(on_btnCancelGoal_clicked()));

    threadpool_.start(action_worker);
}
```

First, this code checks wheater the client is connected to the `shape_server` to make sure it is available.
If this is not the case a [`QMessageBox`](https://doc.qt.io/qt-5/qmessagebox.html) is used to inform the user that the 
`shape_server` in the `turtle_actionlib` should be started with `rosrun turtle_actionlib shape_server`.
Also note that the `turtle_actionlib` works with a turtle named `turtle1`. So make sure that a turtle with that name exists.
To send the goal to another named turtle you can for example run [topic_tools/relay](http://wiki.ros.org/topic_tools/relay).

After the action server is started a new class object of type [`ActionWorker`](https://github.com/fjp/rqt-turtle/blob/master/rqt_turtle/src/rqt_turtle/action_worker.cpp) is created on the heap.
This `ActionWorker` allows us to send the the goal to the shape server without blocking the main gui.
For this [`QThreadpool`](https://doc.qt.io/qt-5/qthreadpool.html) together with [`QRunnable`](https://doc.qt.io/qt-5/qrunnable.html) is used.


```cpp
class ActionWorkerKilledException{};

class ActionWorker : public QObject, public QRunnable
{
    Q_OBJECT

    /// Reference to the action client instanciated in the Draw class.
    actionlib::SimpleActionClient<turtle_actionlib::ShapeAction>& ac_;
    /// Number of edges of the regular polygon
    int edges_;
    /// Radius of the regular polygon
    float radius_;
    /// Duration after which the goal will be canceled and the ActionWorker killed.
    float timeout_;
    /// Defaults to false. Set to true when the kill() slot is called.
    bool is_killed_;

public:
    /**
     * @brief Construct a new Action Worker object
     * 
     * @param ac Reference to the action client which is instanciated in the Draw class that creates this worker.
     * @param edges Specifies the number of edges the worker should draw.
     * @param radius Specifies the shape radius the worker should use while drawing.
     * @param timeout Specifies the duration after which the goal is canceled.
     */
    ActionWorker(actionlib::SimpleActionClient<turtle_actionlib::ShapeAction>& ac, int edges, float radius, float timeout);

    /**
     * @brief Overriede of QRunnable to run the drawing task in another thread from the QThreadPool.
     * 
     */
    void run() override;

signals:
    /**
     * @brief Singals the current progress of the ongoing action. Currently not used
     * 
     * @param value could represent the percentage of the progress.
     */
    void progress(int value);

public slots:
    /**
     * @brief Slot to receive a kill signal when the goal should be canceled.
     * 
     */
    void kill();
};
```

Inside the `ActionWorker::run()` method the current [`actionlib_msgs/GoalStatus`](http://docs.ros.org/noetic/api/actionlib_msgs/html/msg/GoalStatus.html) 
can be checked with the [`actionlib::SimpleActionClient::getState()`](http://docs.ros.org/noetic/api/actionlib/html/classactionlib_1_1SimpleActionClient.html#ac02703c818902c18e72844e58b4ef285) method.
This implementation is similar to the one described in [Writing a Callback Based Simple Action Client](
http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Callback%20Based%20Simple%20Action%20Client).
It avoids blocking until the goal is complete, which would be possible when using [`waitForResult`](https://docs.ros.org/noetic/api/actionlib/html/classactionlib_1_1SimpleActionClient.html#a94b5a38fae6917f83331560b81eea341) described in the [SimpleActionClient] tutorial.

To cancel the goal after the specified timeout is reached the code makes use of [`ros::Time`](http://wiki.ros.org/roscpp/Overview/Time),
[`ros::Duration`](http://docs.ros.org/latest/api/rostime/html/classros_1_1DurationBase.html) and the [possible arithmetic](http://wiki.ros.org/roscpp/Overview/Time#Time_and_Duration_Arithmetic).


While the goal is pursued the `Draw` button changes its text to `CancelGoal` and the connections are updated, see [`Draw::drawShape`](https://github.com/fjp/rqt-turtle/blob/23d7e9430633df276f21c09f17970aa692bf45d2/rqt_turtle/src/rqt_turtle/draw.cpp#L320):

```cpp
ui_->btnDraw->setText(QString("Cancel Goal"));
disconnect(ui_->btnDraw, SIGNAL(clicked()), this, SLOT(on_btnDraw_clicked()));
connect(ui_->btnDraw, SIGNAL(clicked()), action_worker, SLOT(kill()));
connect(ui_->btnDraw, SIGNAL(clicked()), this, SLOT(on_btnCancelGoal_clicked()));
```
Pressing the "new" `Cancel Goal` button reverts the button text and the connections to its original state:

```cpp
void Draw::on_btnCancelGoal_clicked()
{
    ui_->btnDraw->setText("Draw");
    disconnect(ui_->btnDraw, SIGNAL(clicked()), this, SLOT(on_btnCancelGoal_clicked()));
    connect(ui_->btnDraw, SIGNAL(clicked()), this, SLOT(on_btnDraw_clicked()));
}
```

Other related references to [`actionlib`](http://wiki.ros.org/actionlib?distro=noetic):

- [Calling Action Server without Action Client](http://wiki.ros.org/actionlib_tutorials/Tutorials/Calling%20Action%20Server%20without%20Action%20Client)
- [`turtle_actionlib/src/shape_client.cpp`](https://github.com/ros/common_tutorials/blob/noetic-devel/turtle_actionlib/src/shape_client.cpp)
- [Simple ActionClient Threaded](http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionClient%28Threaded%29)


#### Draw Image

With the plugin it's also possible to select an image from disk with the [`QFileDialog`](https://doc.qt.io/qt-5/qfiledialog.html) ([set to the home directory](https://stackoverflow.com/questions/14033720/qfiledialog-how-to-specify-home-directory) by default) and let multiple turtles draw the contours.

![rqt_turtle-draw-image](https://raw.githubusercontent.com/fjp/rqt-turtle/master/docs/rqt_turtle-draw-image-multi.gif)

The contours are found with [`Canny()`](https://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html?highlight=canny#canny) and 
[`findContours`](https://docs.opencv.org/2.4/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html?highlight=findcontours#findcontours) from OpenCV. 
The slider in the gui allows you to set the low threshold of the canny edge detection algorithm which will change the number of detected edges.
A [helpful resource](https://doc.qt.io/qt-5/qobject.html) to convert a `cv::Mat`, used by OpenCV, to a [`QImage`](https://doc.qt.io/qt-5/qimage.html) is this line of code:

```cpp
QImage imgIn= QImage((uchar*) img.data, img.cols, img.rows, img.step, QImage::Format_RGB888);
```

It is used to display the edge image inside the [`Draw.ui`](https://github.com/fjp/rqt-turtle/blob/master/rqt_turtle/resources/Draw.ui) using a QLabel by setting its [`QPixmap`](https://doc.qt.io/qt-5/qpixmap.html) and [scaling it](https://doc.qt.io/qt-5/qpixmap.html#scaled) to the same [size as turtlesim (500x500 pixel)](https://github.com/ros/ros_tutorials/blob/91d2764ebd2344acf8e6754323f5b937b836d218/turtlesim/src/turtle_frame.cpp#L53).
See the [image viewer example](https://doc.qt.io/qt-5/qtwidgets-widgets-imageviewer-example.html) for a good reference on what can be done this way.


For more information on OpenCV Canny and finding contours, see
- [Find contours](https://docs.opencv.org/2.4/doc/tutorials/imgproc/shapedescriptors/find_contours/find_contours.html)
- [tutorial_py_canny](https://docs.opencv.org/trunk/da/d22/tutorial_py_canny.html)
- [canny detector](https://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/canny_detector/canny_detector.html)
- [turotial canny detector](https://docs.opencv.org/3.4/da/d5c/tutorial_canny_detector.html)


To draw the image faster, using multiple turtles, [`QThreadpool`](https://doc.qt.io/qt-5/qthreadpool.html) together with [`QRunnable`](https://doc.qt.io/qt-5/qrunnable.html) is used.
For this, the [`ImageWorker`](https://github.com/fjp/rqt-turtle/blob/master/rqt_turtle/src/rqt_turtle/image_worker.cpp) class inherits from `QRunnable` and [`QObject`](https://doc.qt.io/qt-5/qobject.html) to allow the usage of Qt signals. In this case a `progress(QString name, int progress)`
and `finished(QString name)` is used to update the progress of multiple workers.

The following class shows the header of the `ImageWorker` class. For the source code, please refer to the [`image_worker.cpp`](https://github.com/fjp/rqt-turtle/blob/master/rqt_turtle/src/rqt_turtle/image_worker.cpp).

```cpp
class ImageWorkerKilledException{};


class ImageWorker : public QObject, public QRunnable
{
    Q_OBJECT

    bool is_killed_;

    Turtle turtle_;
    std::vector<std::vector<cv::Point> > contours_;
    int num_contours_;
    int num_points_;
    int idx_contour_;
    int idx_point_;
    int percent_;
    float turtlesim_size_;

    turtlesim::TeleportAbsolute sTeleportAbs_;

public:
    ImageWorker(Turtle turtle, std::vector<std::vector<cv::Point> > contours, float turtlesim_size = 500.0);


    void run() override;

signals:
    void progress(QString name, int value);
    void finished(QString name);

public slots:
    void kill();
};
```

## Install and Run your Plugin

### CMakeLists.txt

Configure the [`CMakeLists.txt`](http://wiki.ros.org/catkin/CMakeLists.txt) using Qt macros, 
see also [Qt's Build with CMake](https://doc.qt.io/qt-5/cmake-manual.html#qt5widgets-macros).
The following macros create rules for calling the [Meta-Object compiler (moc)](https://doc.qt.io/qt-5/moc.html) on the given source/ui file.

- [`qt5_wrap_ui`](https://doc.qt.io/qt-5/qtwidgets-cmake-qt5-wrap-ui.html) the command will configure the project to process `*.ui` 
files into valid C++ `ui_XXXX.h`, [source](https://stackoverflow.com/questions/16245147/unable-to-include-a-ui-form-header-of-qt5-in-cmake), [source](https://stackoverflow.com/questions/19761767/qt-5-cmake-fails-with-undefined-reference-to-vtable-on-hello-world-with-inc-sr).
- [`qt5_wrap_cpp`](https://doc.qt.io/qt-5/qtcore-cmake-qt5-wrap-cpp.html)

Note that these are old and there exist new [`AUTOMOC`](https://cmake.org/cmake/help/v3.0/manual/cmake-qt.7.html#automoc) ([source](https://stackoverflow.com/questions/25875255/cmake-qt5-qt5-wrap-ui-not-generating-ui-header-files)) and [`AUTOUIC`](https://cmake.org/cmake/help/v3.0/manual/cmake-qt.7.html#autouic) options in modern CMake.
{: .notice }

Note: `ui_XXXX.h` files are generated in the build directory `${CMAKE_CURRENT_BINARY_DIR}`. This is why we need to use these `set` commands:

```cmake
# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")

qt5_wrap_ui(rqt_turtle_UIS_H ${rqt_turtle_UIS})

set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")
```

These commands temporarily change the `CMAKE_CURRENT_BINARY_DIR` to `"${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}"` which is a
private include folder inside the devel space. This is done to save the ui header file, created with the meta objec compiler (moc), into the
devel space. TODO confirm this?

```
# TODO use?
find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})
```
### Faced Problems

The original plan was to reuse the service caller and topic caller rqt plugins. However, these plugins are written in Python which
makes it hard to use the ui files because they reference some Python specific Qt widgets that are not available in C++.

To make use of other rqt plugins follow this tutorial and understand [What does `find_package()` do](http://wiki.ros.org/catkin/CMakeLists.txt#Finding_Dependent_CMake_Packages). Also this [answer](https://answers.ros.org/question/201977/include-header-file-from-another-package-indigo/) might be helpful.


The problem with these approaches are that most of the plugins are written in Python.
The [promote widget method](https://doc.qt.io/archives/qt-4.8/designer-using-custom-widgets.html) will work if your plugins use the same programming language because Python doesn't create a header file.

Another problem was that the other plugins don't generate a [PackageNameConfig.cmake](https://cmake.org/cmake/help/v3.0/manual/cmake-packages.7.html#config-file-packages) file which would allow to export the plugin headers and/or their ui files using the [`catkin_package()`](http://wiki.ros.org/catkin/CMakeLists.txt#catkin_package.28.29) macro. 
This is why I just copied the ui files from [`ServiceCaller.ui`](https://github.com/fjp/rqt-turtle/blob/master/rqt_turtle/resources/ServiceCaller.ui) into the resources folder of the `rqt_turtle` plugin and adjusted its elements with the Qt Designer.

Otherwise we could have used

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rqt_gui
  rqt_gui_cpp
  rqt_topic
)
```

or

``` cmake
find_package(rqt_topic)
```


Pro tip: Use `message(${my_package_FOUND})` inside your `CMakeLists.txt` to check if the package was found. If it's found this will output `1` or `True`,
nothing otherwise.
{: notice }

This will define some cmake variables (found [here](https://cmake.org/cmake/help/v3.0/command/find_package.html#find-package)) that make it possible to include the exported resources from that packages using

```cmake
# WIP TODO remove here (only used for rqt_topic)
include_directories(${catkin_INCLUDE_DIRS})
message(${catkin_INCLUDE_DIRS})
message(${rqt_topic_INCLUDE_DIRS})
```


Then, for example using the ui file should work like this:

```cmake
set(rqt_turtle_UIS
  resources/turtle_plugin.ui
  ${rqt_topic_INCLUDE_DIRS}/resource/TopicWidget.ui
)
```


### setup.py

The [setup.py](https://docs.ros.org/noetic/api/catkin/html/user_guide/setup_dot_py.html) is used to install a python package and ROS makes use of this too
if a package contains python scripts.
See [also this tutorial](https://wiki.ros.org/rospy_tutorials/Tutorials/Makefile#Installing_scripts_and_exporting_modules),
[building libraries](http://docs.ros.org/noetic/api/catkin/html/howto/format2/building_libraries.html)
and this [answer](https://answers.ros.org/question/50661/catkin-setuppy-installation-into-devel-space-not-working/)
See [this section](http://wiki.ros.org/rqt/Tutorials/Create%20your%20new%20rqt%20plugin#Install_.26_Run_your_plugin):

See the Running rqt section for how to run your plugin.


With catkin, no matter which method in the link above you want to run your plugin, you need to install it via CMake which puts the script into a package-specific folder which is not on the PATH.

Add macros to your setup.py (reference). For example, after adding the line the section that contains it might look like :

```python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
  
d = generate_distutils_setup(
    packages=['rqt_mypkg'],
    package_dir={'': 'src'},
)

setup(**d)
```

Also make sure in your `CMakeLists.txt`, to uncomment a line:

```cmake
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()
```

Add install macro that puts the script into a location where it is rosrun-able is declared. For example:

```cmake
## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# For our (optional) script to be installed to the right location, 
# if users install your package, this line is required
catkin_install_python(PROGRAMS
  scripts/rqt_turtle
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```


TODO install stuff:




```cmake
#############
## Install ##
#############

# TODO????
# See http://wiki.ros.org/rqt/Tutorials/Create%20your%20new%20rqt%20plugin
# And https://github.com/ros-visualization/rqt_image_view/blob/master/CMakeLists.txt
# TODO?????
#Add the following lines to call the resource and plugin.xml # TODO required?
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)
#install(DIRECTORY
#  resource
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
#install(FILES
#  plugin.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)


## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
```


## Launch the plugin

To launch a rqt plugin make sure to read the [Running rqt User Guide](http://wiki.ros.org/rqt/UserGuide#Running_rqt).

In case it is not working try to use some solutions from [this answer](https://answers.ros.org/question/166851/rqt-reconfigure-no-plugin-found/).


## Future Work

- Add control widgets to publish to the [`/turtleX/cmd_vel`](http://wiki.ros.org/turtlesim#Subscribed_Topics) of type [`geometry_msgs/Twist`](http://docs.ros.org/noetic/api/geometry_msgs/html/msg/Twist.html) topic. 
- Add tests using [rostest](http://wiki.ros.org/rostest/Writing) and [roslaunch test tag](http://wiki.ros.org/roslaunch/XML/test).
- Use `rossrv show` to get the type of a rosservice and use the information for the `request_tree_widget` of the `ServiceCaller.ui`.


## References

- [catkin CMakeLists.txt](http://wiki.ros.org/catkin/CMakeLists.txt)
- [SO: ros-get-current-available-topic-in-code-not-command](https://stackoverflow.com/questions/26785675/ros-get-current-available-topic-in-code-not-command)
- [`rqt_image_view`](https://github.com/ros-visualization/rqt_image_view)
- [ROS `pluginlib`](http://wiki.ros.org/pluginlib)
- [`rqt_service_caller`](http://wiki.ros.org/rqt_service_caller)
- [ROS service list from C++](https://answers.ros.org/question/151611/rosservice-list-and-info-from-c/)
- [ROS Master API](http://wiki.ros.org/ROS/Master_API)
- [TinyXML Tutorial](http://www.grinninglizard.com/tinyxmldocs/tutorial0.html)
- [Qt Designer Using Custom Widgets](https://doc.qt.io/archives/qt-4.8/designer-using-custom-widgets.html)
- [Qt Designer Using a ui file](https://doc.qt.io/qt-5.9/designer-using-a-ui-file.html)
