## Options for path following algorithms

# init_mode: Determines how the path follower reacts to a new path:
#  init_mode = 0 will cause the robot to brake
#  init_mode = 1 will not send a brake command
std_msgs/UInt32 init_mode

# velocity: Driving speed of the robot (interpreted as recommended value)
std_msgs/Float32 velocity

# robot_controller: [optional] name of the controller to use
std_msgs/String robot_controller

# local_planner: [optional] name of the local planner to use
std_msgs/String local_planner

# collision_avoider: [optional] name of the collision avoider to use
std_msgs/String collision_avoider
