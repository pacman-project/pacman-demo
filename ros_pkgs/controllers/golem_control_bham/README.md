GOLEM CONTROLLER BHAM
=====================

This package wraps a Golem controller client. 

Build
-----

It is strongly advised that all packages be built using the main CMakeLists.txt.

However, you can build this package following [this tutorial](http://wiki.ros.org/ROS/Tutorials/BuildingPackages) as a ROS package, but you might need to change the CMakeLists.txt to correct paths.


Use
---

The client can be started with:

`roslaunch golem_control_bham GolemController.launch`

The XML configuration file is specified in the launch file. Note that, this node runs from the folder where the binary is (catkin/devel/golem_control_bham), so if you want to use a different configuration file, you need to copy the file manually there.

A Golem controller server must be started for this package to work, go into the GOLEM_ROOT/bin folder and run

`GolemDeviceCtrlPhysServer` 



Changing the XML configuration files
------------------------------------

You should change the config files in the PACMAN_ROOT/bin folder, and make the pacman project to update the files into the running directory of the ROS nodes.

Memo notes: 

* Check the in the ControlRobotUIBK.xml if you are using the real (GolemDeviceRobotUIBK.xml) or the simulated robot (GolemDeviceRobotUIBKSim.xml), however, in the client side, typically the simulated robot config file is to be used.

* Check the host="ip" to connect to local or remote control servers.

* If you want to run the server in localhost, you need to go into the Golem binaries, and modify the GolemDeviceCtrlPhysServer.xml to point to the simulated robot, since by default it points to the real one.

