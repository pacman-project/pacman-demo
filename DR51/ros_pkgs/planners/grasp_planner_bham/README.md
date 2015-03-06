GRASP PLANNER BHAM
==================

Build
-----

It is strongly advised that all packages be built using the main CMakeLists.txt.

However, you can build this package following [this tutorial](http://wiki.ros.org/ROS/Tutorials/BuildingPackages) as a ROS package, but you might need to change the CMakeLists.txt to correct paths and copy XML configuration files manually.

Use
---

Type:

`roslaunch grasp_planner_bham GraspPlannerBHAM.launch`

The XML configuration file is specified in the launch file. Note that, this node runs from the folder where the binary is (catkin/devel/grasp_planner_bham), so if you want to use a different configuration file, you need to copy the file manually there.

It requires grasp examples. These can be specified in the launch file. For now, only trajectory and one pcd are managed. For more details, see the [Grasp library documentation](https://github.com/pacman-project/Grasp)

Test
----

Changing the XML configuration files
------------------------------------

You should change the config files in the PACMAN_ROOT/bin folder, and make the pacman project to update the files into the running directory of the ROS nodes.

