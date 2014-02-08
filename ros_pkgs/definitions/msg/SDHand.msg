## THIS FILE DEFINES THE SCHUNK DEXTREOUS HAND STATE/COMMAND

geometry_msgs/PoseStamped wrist_pose

# order convention for the joints w.r.t. pacman Defs middle[0,1], left[2,3], right[4,5] rotation[6];
float32[] joints
float32[] velocity
float32[] acceleration
float32[] effort
float32[] stiffness