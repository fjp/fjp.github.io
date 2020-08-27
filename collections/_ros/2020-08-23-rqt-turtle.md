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
  #teaser: /assets/collections/diffbot/assembly/board-plate/03-board-plate-front-left.jpg
  #overlay_image: /assets/collections/diffbot/assembly/board-plate/03-board-plate-front-left.jpg
  overlay_filter: 0.5 # same as adding an opacity of 0.5 to a black background
  caption: rqt_turtle
  show_overlay_excerpt: true
sidebar:
  nav: ros
---

The project shows how the `rqt_turtle` plugin for ROS' [GUI framework rqt](http://wiki.ros.org/rqt) was created. 
The plugin can be used to draw inside [turtlesim](http://wiki.ros.org/turtlesim) with turtlebot.
Although the following description might help you to write your own rqt plugin, also have a look at the official [rqt tutorials](http://wiki.ros.org/rqt/Tutorials). 
There are tutorials explaining how to write plugins using python or C++. The `rqt_turtle` plugin is written in C++.
The [source code](https://github.com/fjp/rqt-turtle) is hosted on GitHub.

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
<library path="lib/">
  <class name="Turtle Plugin" type="rqt_turtle::TurtlePlugin" base_class_type="rqt_gui_py::Plugin">
    <description>
      Plugin for ROS rqt to draw in turtlesim using turtlebot.
    </description>
    <qtgui>
      <!-- optional grouping...
      <group>
        <label>Group</label>
      </group>
      <group>
        <label>Subgroup</label>
      </group>
      -->
      <label>Turtle Plugin</label>
      <icon type="theme">system-help</icon>
      <statustip>Plugin for ROS rqt to draw in turtlesim using turtlebot.</statustip>
    </qtgui>
  </class>
</library>
```

A description of these attributes is found in the [rqt tutorial](http://wiki.ros.org/rqt/Tutorials/Create%20your%20new%20rqt%20plugin#Attributes_of_library_element_in_plugin.xml).


## Write Plugin Code


```cpp
// Deprecated
// See: http://wiki.ros.org/pluginlib#pluginlib.2Fpluginlib_groovy.Simplified_Export_Macro
//PLUGINLIB_DECLARE_CLASS(rqt_turtle, TurtlePlugin, rqt_turtle::TurtlePlugin, rqt_gui_cpp::Plugin)
PLUGINLIB_EXPORT_CLASS(rqt_turtle::TurtlePlugin, rqt_gui_cpp::Plugin)
```

## Install and Run your Plugin

### Configure the `CMakeLists.txt` using Qt macros. Note that these are old and there exist new `AUTOMOC` options in modern CMake.

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
