DEMOS
=====


Build
-----


Use
---

Before launching:  

* check chmod dev sdh, check if robot topics are up, if you are using a real setup.

For the Demo 5.1., start all messages/services/actions by typing

`roslaunch demos demo_5_1_simple.launch`

And then, trigger the logic by typing:

`rosrun demos demo_5_1`

configure your preference parameters in the launch file, like `order_grasp_scene` if you want to consider filter some grasps based on the distance to the safe position or filter grasps close to table due to efficiency in planning
