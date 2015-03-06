DEFINITIONS
===========

This package contains the messages we should be using to communicate between the ROS packages, initially thought to be similar to the PaCMan defs in c++. However, it might be the case that these messages are redundant with respect to the ones already available in ROS.

It is particularly useful to write the demo logic independently of how the messages/services/actions are generated inside the packages, or to write packages to process service requests using the messages/services/actions defined here. However, the use of them is not mandatory, but advised.

Build
-----

It is strongly advised that all packages be built using the main CMakeLists.txt.

However, you can build this package following [this tutorial](http://wiki.ros.org/ROS/Tutorials/BuildingPackages) as a ROS package, but you might need to change the CMakeLists.txt to correct paths.

Use
---

The files generated from the definitions packages are used for interfacing the packages. 