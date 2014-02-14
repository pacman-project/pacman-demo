UIBK Robot MoveIt! Configuration
================================

This package is two-folded:

1. The UIBK configuration with V-Rep and Orocos system. The packages to run it like this are not available in the pacman repos, but in UIBK private repos.

2. The Default configuration with Physx and Golem system. The packages to run it like this are available in the pacman repos. (in Process)

Build
-----

It is strongly advised that all packages be built using the main CMakeLists.txt.

However, you can build this package following [this tutorial](http://wiki.ros.org/ROS/Tutorials/BuildingPackages) as a ROS package, but you might need to change the CMakeLists.txt to correct paths.

Use
---

1. ToDo: describe the V-Rep/Orocos configuration

2. To launch the moveit! environment and the path planner c++ interface, type: 

term1:

`roslaunch uibk_robot_moveit_config uibk_robot_path_planner.launch`

Some basic parameters for the planner can be set in the `launch/uibk_robot_path_planner.launch` file. For more other parameters, you can use the moveit! GUI.

NOTE: right now the default tolerance for position and orientation is set 0.1, because with 0.001 is hard to find solutions.  Check this, because 0.1 is too high.

Test
----

You can test the planner by typing :

`rosrun uibk_robot_moveit_config ping_path_planner`

After launching, and this will plan to wrist which is known to have solution.