PLANNERS
========


UPDATE: both approaches are stacked now, first 2 is used, and if not solution found, then 1 is used.

At the moment there are two possibilities for CartPlanner: 
1- using inverse kinematics solver   CartPlanner_uibk
2- using waypoints for producing smooth trajectorie CartPlanner

select the desired CartPlanner in the CMakeFile of planner
in the target_link_libraries for trajectory_planner_node, enable the desired CartPlanner
in trajectory_planner_node uncomment the desired CartPlanner and comment the other one.

