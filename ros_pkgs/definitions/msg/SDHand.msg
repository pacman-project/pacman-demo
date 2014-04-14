## THIS FILE DEFINES THE SCHUNK DEXTREOUS HAND STATE/COMMAND

geometry_msgs/PoseStamped wrist_pose

# order convention for the joints w.r.t. pacman Defs rotation[0], left[1,2], right[3,4], thumb[5,6] ;
float32[] joints
float32[] velocity
float32[] acceleration
float32[] effort
float32[] stiffness