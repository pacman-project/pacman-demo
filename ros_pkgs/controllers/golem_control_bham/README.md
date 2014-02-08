GOLEM CONTROLLER BHAM
=====================

BUILD
-----

Before compiling, set the pacman root folder on the CMakeLists.txt

USING
-----

Be sure you have the latest XML files in the config folder. The handling of this files need to be improved, but for now, it is ok.

A Golem controller server must be started for this package to work.

Check the in the ControlRobotUIBK.xml if you are using the real (GolemDeviceRobotUIBK.xml) or the simulated robot (GolemDeviceRobotUIBKSim.xml), however, in the client side, typically the simulated robot config file is to be used.

And check within them, the host="ip" to connect to local or remote control servers.
