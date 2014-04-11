GOLEM CONTROLLER BHAM
=====================

This package wraps a Golem controller client. 

Build
-----

It is strongly advised that all packages be built using the main CMakeLists.txt.

However, you can build this package following [this tutorial](http://wiki.ros.org/ROS/Tutorials/BuildingPackages) as a ROS package, but you might need to change the CMakeLists.txt to correct paths.


Use
---

A Golem controller server must be started for this package to work, go into the GOLEM_ROOT/bin folder and run

`GolemDeviceCtrlPhysServer` 



The client for the real robot can be started with:

`roslaunch golem_control_bham GolemController.launch`

The client for the simulated robot can be started with:

`roslaunch golem_control_bham GolemControllerSim.launch`

The XML configuration file is specified in the launch file. Note that, this node runs from the folder where the binary is (catkin/devel/golem_control_bham), see the section below if you want to change the configuration files.



Changing the XML configuration files
------------------------------------

You should change the config files in the PACMAN_ROOT/bin folder, and make the pacman project to update the files into the running directory of the ROS nodes.

Memo notes: 

* Check the host="ip" to connect to local or remote control servers.

* If you want to run the server in localhost, you need to go into the Golem binaries, and modify the GolemDeviceCtrlPhysServer.xml to point to the simulated robot, since by default it points to the real one.

