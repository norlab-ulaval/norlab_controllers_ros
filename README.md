# Norlab controllers ROS

## Description
Norlab controllers ROS is a set of ROS packages enabling path following for uncrewed ground vehicles (UGVs).
This repository also includes the [`norlab_controllers`](https://github.com/norlab-ulaval/norlab_controllers) python library providing an implementation of various popular controllers.
The goal of this repository is to offer a lean and modular solution for UGV path following.

For installation, you need to clone the repository in your ROS 2 workspace, initialize its submodules and install the library:

```bash
cd <PATH_TO_YOUR_ROS2_WORKSPACE>/src/
git clone git@github.com:norlab-ulaval/norlab_controllers_ros.git
git submodule init
git submodule update
cd norlab_controllers_ros/norlab_controllers/
./install.sh
cd <PATH_TO_YOUR_ROS2_WORKSPACE>/src/
colcon build --symlink-install
source install/local_setup.bash
```

## Usage

The main package, [`norlab_controllers_wrapper`](https://github.com/norlab-ulaval/norlab_controllers_ros/tree/humble/norlab_controllers_wrapper) acts as a [ROS2 action server](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html).
The server recieves target paths as actions sent by the client, these custom actions are defined in the [`norlab_controllers_msgs`](https://github.com/norlab-ulaval/norlab_controllers_ros/tree/humble/norlab_controllers_msgs) package.
Refer to our [`WILN`](https://github.com/norlab-ulaval/wiln) package for a functional implementation of a action client.

## Launching the nodes

A set of launch files are provided in the [`norlab_controllers_wrapper`](https://github.com/norlab-ulaval/norlab_controllers_ros/tree/humble/norlab_controllers_wrapper) package.
These launch files allow to run the path-following nodes:
```bash
ros2 launch norlab_controllers_wrapper warthog_wheels_mpc_launch.xml
```
For quick bootstrapping, the different services of [`WILN`](https://github.com/norlab-ulaval/wiln) can be used to perform teach-and-repeat operation.

## Tuning the controller's parameters

Controller parameters are defined and can be modified in the [`params`](https://github.com/norlab-ulaval/norlab_controllers_ros/tree/humble/norlab_controllers_wrapper/params) folder. 
Please note that the [`norlab_controllers_inspector`](https://github.com/norlab-ulaval/norlab_controllers_inspector/tree/master) could then be used to visualize and quantify the controller performance. 
Controller parameters files are in the [YAML](https://en.wikipedia.org/wiki/YAML) format.
For faster tuning, you can build your ROS2 workspace with symbolic links to prevent the requirement to rebuild the workspace anytime you change a parameter or change a script.
```bash
colcon build --symlink-install
```





