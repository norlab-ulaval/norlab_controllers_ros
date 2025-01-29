#!/usr/bin/env python3

import os
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_action_status_default
from multiprocessing import Lock

from geometry_msgs.msg import Twist, TwistStamped, PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as Ros2Path
from std_msgs.msg import UInt32,Float32

from tf2_ros import Buffer, TransformListener

from norlabcontrollib.path.path import Path
from norlabcontrollib.controllers.controller_factory import ControllerFactory
from norlab_controllers_msgs.action import FollowPath
from rcl_interfaces.msg import SetParametersResult
import yaml

from scipy.spatial.transform import Rotation as R
import time


class ControllerNode(Node):

    def __init__(self):
        super().__init__("controller_node")

        # TODO: Check if these lines are necessary
        abspath = os.path.abspath(__file__)
        dname = os.path.dirname(abspath)
        os.chdir(dname)
        self.get_logger().info(os.getcwd())

        controller_config_path = self.declare_parameter("controller_config", "None").value
        self.get_logger().info(f"Controller config: {controller_config_path}")
        
        rotation_controller_config_path = self.declare_parameter("rotation_controller_config", "None").value
        self.get_logger().info(f"Rotation controller config: {rotation_controller_config_path}")

        self.controller_factory = ControllerFactory()
        self.controller = self.controller_factory.load_parameters_from_yaml(
            controller_config_path
        )

        if rotation_controller_config_path == "None":
            self.rotation_controller_bool = False
        else:
            self.rotation_controller = self.controller_factory.load_parameters_from_yaml(
                rotation_controller_config_path
            )
            self.rotation_controller_bool = True

        # Add the dynamic parameter
        self.init_params(controller_config_path)

        # Initialize state and velocity
        self.state = np.zeros(6)  # [x, y, z, roll, pitch, yaw]
        self.velocity = np.zeros(6)  # [vx, vy, vz, v_roll, v_pitch, v_yaw]
        self.state_velocity_mutex = Lock()

        # Initialize publishers
        # self.cmd_publisher_ = self.create_publisher(TwistStamped, "cmd_vel_out", 100)
        self.cmd_publisher_ = self.create_publisher(Twist, "cmd_vel_out", 100)
        self.optim_path_publisher_ = self.create_publisher(
            Ros2Path, "optimal_path", 100
        )
        self.target_path_publisher_ = self.create_publisher(
            Ros2Path, "target_path", 100
        )
        self.ref_path_publisher_ = self.create_publisher(
            Ros2Path, "ref_path", qos_profile_action_status_default,  # Makes durability transient_local
        )

        # Initialize subscribers
        # self.odom_subscription = self.create_subscription(
        #     Odometry, "odom_in", self.odometry_callback, 10
        # )

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.tf_timer = self.create_timer(1/self.controller.rate, self.update_robot_pose)

        # Odom timer
        self.odom_timer = self.create_timer(1.0, self.update_odom_counter)
        self.odom_counter = 0

        # Initialize action server
        self._action_server = ActionServer(
            self, FollowPath, "/follow_path", self.follow_path_callback, cancel_callback=self.cancel_callback
        )

        self.rate = self.create_rate(self.controller.rate)

        self.waiting_for_path = True
        self.loading_path = False
        self.executing_path = False

        self.last_compute_time = self.get_clock().now().nanoseconds * 1e-9
        self.last_odom_time = self.get_clock().now().nanoseconds * 1e-9

        self.angular_goal_bool_pub = self.create_publisher(Float32, 'angular_distance_2_goal', 10)
        self.distance_goal_bool_pub = self.create_publisher(Float32, 'distance_2_goal', 10)
        self.timer = self.create_timer(0.5, self.publish_distance_2_goal)



    def init_params(self, yaml_file_path):

        # Get dict format of the parameter
        with open(yaml_file_path) as yaml_file:
            yaml_params = yaml.full_load(yaml_file)
            self.get_logger().info(str(yaml_params))

            for init_param, init_value in yaml_params.items():
                param = self.declare_parameter(init_param, init_value)
                self.get_logger().info(f"{param.name}={param.value}")

        self.add_on_set_parameters_callback(self.on_params_changed)

    def on_params_changed(self, params):
        
        # Change controller parameters during runtime (dynamic parameters)
        current_params = self.controller.__dict__

        for param in params:
            if param.name in current_params.keys():
                param_in_controller = self.controller.__dict__[param.name]
                self.get_logger().info(
                    f"Trying to change [{param.name}] to {param.value}, was {param_in_controller}."
                )

                current_params[param.name] = param.value
                self.controller.__dict__[param.name] = param.value
                self.controller.init_casadi_model()

                param_in_controller = self.controller.__dict__[param.name]
                self.get_logger().info(
                    f"The param [{param.name}] has been set to {param_in_controller}"
                )

            else:
                continue

        return SetParametersResult(successful=True, reason="Parameter set")

    def publish_distance_2_goal(self):

        self.distance_goal_bool_pub.publish(Float32(data=float(self.controller.distance_to_goal)))
        self.angular_goal_bool_pub.publish(Float32(data=float(self.controller.angular_distance_to_goal)))

    def odometry_callback(self, message):
        self.last_odom_time = self.get_clock().now().nanoseconds * 1e-9
        self.odom_counter += 1
        with self.state_velocity_mutex:
            position = message.pose.pose.position
            quat = message.pose.pose.orientation
            twist = message.twist.twist
            self.state[0:3] = [position.x, position.y, position.z]
            self.state[3:] = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler("xyz")
            self.velocity[0:3] = [twist.linear.x, twist.linear.y, twist.linear.z]
            self.velocity[3:] = [twist.angular.x, twist.angular.y, twist.angular.z]

    def update_odom_counter(self):
        if self.odom_counter < 8:
            self.get_logger().warn(f"Received only {self.odom_counter} odom messages in the last second!")
        self.odom_counter = 0

    def update_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            position = tf.transform.translation
            quat = tf.transform.rotation
            self.state[0:3] = [position.x, position.y, position.z]
            self.state[3:] = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler("xyz")
            self.last_odom_time = self.get_clock().now().nanoseconds * 1e-9
            self.odom_counter += 1  
        except:
            self.get_logger().warn("Failed to get transform from base_link to map.")

    def command_array_to_twist_msg(self, command_array):
        # cmd_vel_msg = TwistStamped()
        # cmd_vel_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = command_array[0]
        cmd_vel_msg.angular.z = command_array[1]
        return cmd_vel_msg

    def planar_state_to_pose_msg(self, planar_state):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position = Point(x=planar_state[0], y=planar_state[1], z=0.0)
        x, y, z, w = R.from_euler('xyz', [0.0, 0.0, planar_state[2]]).as_quat()
        pose_msg.pose.orientation = Quaternion(x=x, y=y, z=z, w=w)
        return pose_msg

    def compute_then_publish_command(self):
        with self.state_velocity_mutex:
            if self.last_compute_time < self.last_odom_time:
                start_time = time.time()
                command_vector = self.controller.compute_command_vector(self.state)
                self.get_logger().info(f"Computing delay: {time.time() - start_time} sec")
                self.last_compute_time = self.get_clock().now().nanoseconds * 1e-9
                self.get_logger().info("COMPUTING!")
            else:
                command_vector, id = self.controller.get_next_command()
                self.get_logger().info(f"Executing next command, {id}.")
            cmd_vel_msg = self.command_array_to_twist_msg(command_vector)
            self.cmd_publisher_.publish(cmd_vel_msg)

    def compute_then_publish_rotation_command(self):
        with self.state_velocity_mutex:
            command_vector = self.rotation_controller.compute_command_vector(self.state)
            cmd_vel_msg = self.command_array_to_twist_msg(command_vector)
            self.cmd_publisher_.publish(cmd_vel_msg)

    def publish_optimal_path(self):
        optim_path_msg = Ros2Path()
        optim_path_msg.header.stamp = self.get_clock().now().to_msg()
        optim_path_msg.header.frame_id = "map"
        for k in range(0, self.controller.horizon_length):
            pose = self.planar_state_to_pose_msg(
                self.controller.optim_trajectory_array[:, k]
            )
            pose.pose.position.z = 0.1
            optim_path_msg.poses.append(pose)
        self.optim_path_publisher_.publish(optim_path_msg)

    def publish_target_path(self):
        target_path_msg = Ros2Path()
        target_path_msg.header.stamp = self.get_clock().now().to_msg()
        target_path_msg.header.frame_id = "map"
        for k in range(0, self.controller.horizon_length):
            pose = self.planar_state_to_pose_msg(
                self.controller.target_trajectory[:, k]
            )
            pose.pose.position.z = 0.05
            target_path_msg.poses.append(pose)
        self.target_path_publisher_.publish(target_path_msg)

    def publish_reference_path(self):
        ref_path_msg = Ros2Path()
        ref_path_msg.header.stamp = self.get_clock().now().to_msg()
        ref_path_msg.header.frame_id = "map"
        for k in range(0, self.controller.path.n_poses):
            planar_state = [
                self.controller.path.poses[k, 0],
                self.controller.path.poses[k, 1],
                self.controller.path.angles[k],
            ]
            pose = self.planar_state_to_pose_msg(planar_state)
            ref_path_msg.poses.append(pose)
        self.ref_path_publisher_.publish(ref_path_msg)

    def follow_path_callback(self, goal_handle):
        ## Importing all goal paths
        self.get_logger().info("Importing goal paths...")
        current_path = self.custom_path_from_ros2(goal_handle.request.path)
        self.controller.update_path(current_path)
        self.publish_reference_path()

        self.get_logger().info(f"Path import done, proceeding to executing {current_path.n_poses} pose(s)...")

        self.controller.previous_input_array = np.zeros((2, self.controller.horizon_length))
        self.controller.compute_distance_to_goal(self.state, 0)
        self.last_distance_to_goal = self.controller.distance_to_goal
        self.controller.next_path_idx = 0

        self.get_logger().info(f"Initial state: {self.state}")
        self.get_logger().info(f"Ref path: {self.controller.path.poses}")
        self.get_logger().info(f"Distance to goal: {self.controller.distance_to_goal} m.")

        while not self.controller.goal_reached():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled! Stopping robot.')
                self.stop_robot()
                self.clear_paths()
                return FollowPath.Result()
            self.get_logger().info(f"Distance to goal: {self.controller.distance_to_goal} m.")
            self.get_logger().debug(f"Angular distance_to_goal: {self.controller.angular_distance_to_goal}")
            self.compute_then_publish_command()
            self.publish_optimal_path()
            self.publish_target_path()
            self.print_debug()
            if self.controller.next_path_idx >= self.controller.path.n_poses - 1:
                if self.controller.distance_to_goal > self.last_distance_to_goal:
                    break
                else:
                    self.last_distance_to_goal = self.controller.distance_to_goal
            self.rate.sleep()

            self.get_logger().info(str(self.controller.debug_indicator))
        self.get_logger().info("SUCCESS")
        self.clear_paths()

        ## return completed path to action client
        goal_handle.succeed()

        # Stop the robot 
        self.stop_robot()
        
        #send results
        paths_result = FollowPath.Result()
        paths_result.result_status = UInt32(data=1)
        return paths_result
    
    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def stop_robot(self):
        # self.cmd_vel_msg = TwistStamped()
        # self.cmd_vel_msg.header.stamp = self.get_clock().now().to_msg()
        self.cmd_vel_msg = Twist()
        self.cmd_publisher_.publish(self.cmd_vel_msg)
    
    def custom_path_from_ros2(self, ros2_path):
        poses = []
        for pose in ros2_path.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            z = pose.pose.position.z
            quat = pose.pose.orientation
            roll, pitch, yaw = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler("xyz")
            poses.append([x, y, z, roll, pitch, yaw])
        print(poses)
        return Path(np.array(poses))
    
    def clear_paths(self):
        empty_path_msg = Ros2Path()
        self.ref_path_publisher_.publish(empty_path_msg)
        self.target_path_publisher_.publish(empty_path_msg)
        self.optim_path_publisher_.publish(empty_path_msg)

    def print_debug(self):
        self.get_logger().debug(
            f"Next command : (Left) {self.controller.optimal_left}, (Right) {self.controller.optimal_right}"
        )
        self.get_logger().debug(f"Planar state : {self.controller.planar_state}")
        self.get_logger().debug(f"Target path: {self.controller.target_trajectory.T}")
        for j in range(0, self.controller.horizon_length):
            self.get_logger().debug(
                f"optimal_left_{j} {self.controller.optim_solution_array[j]}"
            )
            self.get_logger().debug(
                f"optimal_right_{j} {self.controller.optim_solution_array[j + self.controller.horizon_length]}"
            )
        # self.get_logger().debug('Path Curvature : ' + str(self.controller.path_curvature))
        self.get_logger().debug(
            f"look ahead distance counter: {self.controller.path_look_ahead_distance}"
        )
        self.get_logger().debug(f"Distance_to_goal: {self.controller.distance_to_goal}")
        self.get_logger().debug(f"Angular distance_to_goal: {self.controller.angular_distance_to_goal}")
        #self.get_logger().debug(
        #    f"Euclidean Distance_to_goal: {self.controller.distance_to_goal}"
        #)


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)

    try:
        # declare the node constructor
        controller_node = ControllerNode()
        executor = MultiThreadedExecutor()
        executor.add_node(controller_node)

        try:
            # pause the program execution, waits for a request to kill the node (ctrl+c)
            executor.spin()
        finally:
            executor.shutdown()
            controller_node.destroy_node()
    finally:
        # shutdown the ROS communication
        rclpy.shutdown()


if __name__ == "__main__":
    main()
