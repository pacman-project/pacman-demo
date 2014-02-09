UIBK Robot MoveIt! Configuration
================================

This package is two-folded:

1. The UIBK configuration with V-Rep and Orocos system. The packages to run it like this are not available in the pacman repos, but in UIBK private repos.

2. The Default configuration with Physx and Golem system. The packages to run it like this are available in the pacman repos. (in Process)

BUILD
-----


USING
-----

2. For now, you can run 

term1:

`roslaunch uibk_robot_moveit_config demo.launch`

term2:

`rosrun uibk_robot_moveit_config path_planner_node`

To test, go to term3 (this will plan to wrist which is known to have solution):

`rosrun uibk_robot_moveit_config ping_path_planner`


ToDo: launch file for term1, term2, and have access to the planning parameters trough the launch file.

NOTE: right now the default tolerance for position and orientation is set 0.1, because with 0.001 is hard to find solutions.  Check this, because 0.1 is too high.
