# Norlab controllers ROS

## Description
The Norlab's robots are collecting datasets and capturing Light Detection and Ranging (LiDAR) maps in subarctic forests using an autonomous teach-and-repeat system designed to be robust to kilometer-scale navigation, severe weather, and GNSS-denied conditions. During the teach phase, the robot is driven along a specific path by a human operator. A reference map and a reference path are meanwhile recorded and stored in the robot’s database. Then, during the repeat phase, a path following algorithm computes the output system commands to steer the vehicle along the reference path. This path following algorithm used is a Model Predictive Control (MPC).

The implementation of this MPC algorithm is divided into two parts: a Robot Operating System (ROS) repository named [norlab_controllers_ros](https://github.com/norlab-ulaval/norlab_controllers_ros) and a python library named [norlab_controllers](https://github.com/norlab-ulaval/norlab_controllers). The ROS repository receives the robot's current position and target path from other ROS perception ([mapping repository](https://github.com/norlab-ulaval/husky_mapping)) and planning ([wiln repository](https://github.com/norlab-ulaval/wiln))  nodes, and sends commands to the motors. However, this ROS repository does not handle the optimization computations required to operate the controller. These calculations are performed by the Python library, which is not part of the robot's ROS architecture. This library has deliberately been placed outside the ROS architecture, to make it easier to use in other contexts. The latter is responsible for finding the argument that minimizes a cost function, corresponding to the optimization problem. This optimization is performed using the [CasADi](https://web.casadi.org/) open-source tool specialized in optimal control. Using symbolic programming and algorithmic differentiation, this tool solves the optimization problem in around 6 ms.

## Quick start 
Once the teach-and-repeat framework and the mapping nodes are running (using respectively the [wiln launch file](https://github.com/norlab-ulaval/wiln/blob/humble/launch/marmotte.launch.xml) FIXME BAD LINK HUSKY) and the [mapping launch file](https://github.com/norlab-ulaval/husky_mapping/blob/humble/launch/realtime_experiment.launch.xml)), the following command may be executed to start the controller:
```bash
ros2 launch norlab_controllers_ros husky_mpc_launch.xml 
```
Then, the different services of [wiln](https://github.com/norlab-ulaval/wiln) are used to perform teach-and-repeat operation.

## Tuning the controller's parameters
If you wish to change the controller's parameters, to increase its performance on a particular environment for instance, you may do so in the [params folder](https://github.com/norlab-ulaval/norlab_controllers_ros/tree/humble/params). Please note that the [norlab_controllers_inspector](https://github.com/norlab-ulaval/norlab_controllers_inspector/tree/master) could then be used to visualize and quantify the controller performance. First, you will need to navigate to the params folder from a robot terminal by running the following command:
```bash
cd workspaces/norlab_ws/src/norlab_controllers_ros/params
```
Then, to modify the appropriate parameters file, run the following command:
```bash
vim husky-mpc.yaml
```
Once you have made your changes, get back to the /workspaces repository and execute the following line to build the packages and incorporate your changes into the controller.
```bash
./symlink_build.sh
```





