HARDWARE
========

This meta-package contains the geometric and dynnamic description of the available hardware. Any change in the geometry of the robots should be updated here, and not in the c++ code, such as adding connectors, mounting different hands, and so on.

ToDo: create the KIT head description
ToDo: copy from Pisa repos the description of the DLR/HIT and Soft/IIT hand (still waiting for the newest hardware version that will be send to partners). 
ToDo: create a folder called `pacman_devices` that contains the description of separate elements, such as a kuka arm, dlr hand, schunk gripper, kit head, uibk/unipi/bham torsos, rgbd sensor, etc. and a folder called `pacman_robots` which assemble all this devices in different configurations. 

Folders
-------


Build
-----

There should be nothing to be built within this package.

Use
---

`roslaunch YOUR_HW_PACKAGE display.launch`

Adding new hardware
-------------------

1. Create the (meta-)package of your new hardware called YOUR_HW_PACKAGE

2. Create a xacro definition of your model macro within `xacro` folder callled `YOUR_MODEL.xacro`.

3. (optional) Create a xacro file called `YOUR_EXAMPLE.urdf.xacro` within the `xacro` folder that shows an example on how to instantiate `YOUR_MODEL.xacro` model.

4. (optional) Create a launch file called `display.launch` within the `launch` folder that displays your example as:

```
<launch>
	<arg name="gui" default="True" />
	<param name="use_gui" value="$(arg gui)"/>
	<param name="robot_description" command="$(find xacro)/xacro.py $(find YOUR_HW_PACKAGE)/xacro/YOUR_EXAMPLE.urdf.xacro" />
	<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" ></node>
	<node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" />
</launch>
```