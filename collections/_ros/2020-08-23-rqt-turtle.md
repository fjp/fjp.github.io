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
# classes: wide
header:
  teaser: /assets/ros/rqt_turtle/turtle_plugin_ui.png"><img src="/assets/ros/rqt_turtle/turtle_plugin_ui.png
  overlay_image: /assets/ros/rqt_turtle/turtle_plugin_ui.png"><img src="/assets/ros/rqt_turtle/turtle_plugin_ui.png
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
The plugin can be used to draw inside [turtlesim](http://wiki.ros.org/turtlesim) with turtlebot.
Although the following description might help you to write your own rqt plugin, also have a look at the official [rqt tutorials](http://wiki.ros.org/rqt/Tutorials). 
There are tutorials explaining how to write plugins using python or C++. The `rqt_turtle` plugin is written in C++.
The [source code](https://github.com/fjp/rqt-turtle) is hosted on GitHub.

## C++ vs Python

Note that most rqt plugins are wirtten in Python and it is even recommended to write them in Python. 
However, because of few documented C++ plugins, this project shows how to write a plugin in C++ and it is similar to the
[`rqt_image_view` plugin](http://wiki.ros.org/rqt_image_view) that is also programmed in C++. 
In case you are planning to reuse existing rqt plugins make sure to use the 
language that they are written in. As you will see later in this writeup, I had to use some compromise to use existing rqt plugins which are were written in Python.


## Create Empty ROS Package

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

## Modify package.xml

To make the rqt plugin discoverable, for catkin, you must declare the plugin in the `package.xml` by referencing a `plugin.xml`.

```xml
  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->
    <rqt_gui plugin="${prefix}/plugin.xml"/>
  </export>
```

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

A description of these attributes is found in the [rqt tutorial](http://wiki.ros.org/rqt/Tutorials/Create%20your%20new%20rqt%20plugin#Attributes_of_library_element_in_plugin.xml).


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


```cpp
// Deprecated
// See: http://wiki.ros.org/pluginlib#pluginlib.2Fpluginlib_groovy.Simplified_Export_Macro
//PLUGINLIB_DECLARE_CLASS(rqt_turtle, TurtlePlugin, rqt_turtle::TurtlePlugin, rqt_gui_cpp::Plugin)
PLUGINLIB_EXPORT_CLASS(rqt_turtle::TurtlePlugin, rqt_gui_cpp::Plugin)
```

### Service Caller

ROS provides the `rosservice` tool which has the `list` subcommand to list all the available services that are 
currently registered with the ROS master. To get the service list in our C++ code we can make use
of XMLRPC which ROS uses under the hood for its communication. With that we can query the 
ROS master through its [ROS Master API](http://wiki.ros.org/ROS/Master_API).

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

To access the relevant part, the service list, we can use the following for loop:

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

{::options parse_block_html="true" /}
<details><summary markdown='span'>CLICK ME CC</summary>

## yes, even hidden code blocks!

```python
print("hello world!")
```

</details>


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



Useful references for working with XMLRPC in the roscpp client library are this [answer](https://answers.ros.org/question/151611/rosservice-list-and-info-from-c/?answer=152421#post-id-152421) and the [ROS Master API Wiki page](http://wiki.ros.org/ROS/Master_API).


## Install and Run your Plugin

### CMakeLists.txt

Configure the `CMakeLists.txt` using Qt macros. Note that these are old and there exist new `AUTOMOC` options in modern CMake.

Helpful resources:
- [SO](https://stackoverflow.com/questions/16245147/unable-to-include-a-ui-form-header-of-qt5-in-cmake)
- [`qt5_wrap_ui`](https://doc.qt.io/qt-5/qtwidgets-cmake-qt5-wrap-ui.html)
- [`qt5_wrap_cpp`](https://doc.qt.io/qt-5/qtcore-cmake-qt5-wrap-cpp.html)

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


To make use of other rqt plugins follow this tutorial and understand [What does `find_package()` do](http://wiki.ros.org/catkin/CMakeLists.txt#Finding_Dependent_CMake_Packages). Also this [answer](https://answers.ros.org/question/201977/include-header-file-from-another-package-indigo/) might be helpful.


The problem with these approaches are that most of the plugins are written in Python.
The promote widget method will work if your plugins use the same programming language because Python doesn't create a header file.

Another problem was that the other plugins don't generate a Cmake.config file which would allow to export the plugin headers and/or their ui files using the [`catkin_package()`](http://wiki.ros.org/catkin/CMakeLists.txt#catkin_package.28.29) macro. 
This is why I just copied the ui files from `ServiceCaller.ui` and ... into the resources folder of the `rqt_turtle` plugin.

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

to include the resources from other plugins.

This will define some cmake variables (found here) that make it possible to include the exported resources from that packages using

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

See [also this tutorial](https://wiki.ros.org/rospy_tutorials/Tutorials/Makefile#Installing_scripts_and_exporting_modules)
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


## References

- [catkin CMakeLists.txt](http://wiki.ros.org/catkin/CMakeLists.txt)
- [SO: ros-get-current-available-topic-in-code-not-command](https://stackoverflow.com/questions/26785675/ros-get-current-available-topic-in-code-not-command)
- [`rqt_image_view`](https://github.com/ros-visualization/rqt_image_view)
- [ROS `pluginlib`](http://wiki.ros.org/pluginlib)
- [`rqt_service_caller`](http://wiki.ros.org/rqt_service_caller)
- [ROS service list from C++](https://answers.ros.org/question/151611/rosservice-list-and-info-from-c/)
- [ROS Master API](http://wiki.ros.org/ROS/Master_API)
- [TinyXML Tutorial](http://www.grinninglizard.com/tinyxmldocs/tutorial0.html)
