#!/usr/bin/env python3

import os
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_action_status_default
from multiprocessing import Lock

from geometry_msgs.msg import TwistStamped, PoseStamped, Point, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import UInt32, Float32
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import Buffer, TransformListener

from norlabcontrollib.path.path import Path as CustomPath
from norlabcontrollib.controllers.controller_factory import ControllerFactory
from norlabcontrollib.controllers.mppi import MPPI
from controller_msgs.action import FollowPath
from rcl_interfaces.msg import SetParametersResult
import yaml

from scipy.spatial.transform import Rotation as R


class ControllerNode(Node):

    def __init__(self):
        super().__init__("controller_node")

        # Initialize parameters, subscribers, publishers, timers
        self.init_parameters()
        self.init_subscribers()
        self.init_publishers()
        self.init_timers()

        # Initialize state and command
        self.state = np.zeros(6)  # [x, y, z, roll, pitch, yaw]
        self.state_mutex = Lock()
        self.cmd_vel_msg = TwistStamped()

        # Initialize action server
        self._action_server = ActionServer(
            self, FollowPath, self.follow_path_topic, self.follow_path_callback, cancel_callback=self.cancel_callback
        )

        self.rate = self.create_rate(self.controller.rate)
        self.last_compute_time = 0.0
        self.last_tf_time = 0.0

        # Fraction of MPPI sampled trajectories drawn in the "sampled_trajectories" marker
        # (matches sampled_trajectories_plot_fraction in mppi.py's own debug plot).
        self.sampled_trajectories_plot_fraction = 0.1

    def init_parameters(self):

        self.controller_config = self.declare_parameter("controller_config", "").value
        self.controller = ControllerFactory().load_parameters_from_yaml(self.controller_config)

        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.robot_frame = self.declare_parameter("robot_frame", "base_link").value
        self.follow_path_topic = self.declare_parameter("follow_path_topic", "follow_path").value

        self.get_logger().info("Controller parameters:")
        with open(self.controller_config) as yaml_file:  # type: ignore
            yaml_params = yaml.full_load(yaml_file)
            for init_param, init_value in yaml_params.items():
                param = self.declare_parameter(init_param, init_value)
                self.get_logger().info(f"   {param.name} = {param.value}")

        self.add_on_set_parameters_callback(self.update_parameters)

    def update_parameters(self, params):

        # Change controller parameters during runtime (dynamic parameters)
        current_params = self.controller.__dict__

        for param in params:
            if param.name in current_params.keys():
                self.get_logger().info(
                    f"Trying to change [{param.name}] to {param.value}, was {self.controller.__dict__[param.name]}."
                )

                self.controller.__dict__[param.name] = param.value
                if hasattr(self.controller, "init_casadi_model"):
                    self.controller.init_casadi_model()

                self.get_logger().info(
                    f"The param [{param.name}] has been set to {self.controller.__dict__[param.name]}"
                )
            else:
                continue

        return SetParametersResult(successful=True, reason="Parameter set")

    def init_subscribers(self):

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

    def init_publishers(self):

        self.command_pub = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.optimal_path_pub = self.create_publisher(Path, "optimal_path", 100)
        self.target_path_pub = self.create_publisher(Path, "target_path", 100)
        self.reference_path_pub = self.create_publisher(
            Path, "ref_path", qos_profile_action_status_default
        )  # Makes durability transient_local
        self.sampled_trajectories_pub = self.create_publisher(MarkerArray, "sampled_trajectories", 10)
        self.goal_linear_distance_pub = self.create_publisher(Float32, "linear_distance_to_goal", 10)
        self.goal_angular_distance_pub = self.create_publisher(Float32, "angular_distance_to_goal", 10)

    def init_timers(self):

        self.tf_timer = self.create_timer(1 / self.controller.rate, self.update_robot_pose)
        self.distance_timer = self.create_timer(0.5, self.publish_distance_to_goal)

    def update_robot_pose(self):

        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rclpy.time.Time())  # type: ignore
            position = tf.transform.translation
            quat = tf.transform.rotation
            self.state[0:3] = [position.x, position.y, position.z]
            self.state[3:] = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler("xyz")

            self.last_tf_time = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            if self.get_clock().now().nanoseconds * 1e-9 - self.last_tf_time > 1.0:
                self.get_logger().warn("The last TF message is older than 1 second!")

        except Exception as e:
            self.get_logger().log(
                f"Failed to get transform: {e}", rclpy.logging.LoggingSeverity.WARN, throttle_duration_sec=1.0  # type: ignore
            )

    def follow_path_callback(self, goal_handle):

        self.get_logger().info("Received path to follow.")

        if self.last_tf_time == 0.0:
            self.get_logger().warn("No TF received yet, cannot start following path.")
            goal_handle.abort()
            return FollowPath.Result(result_status=UInt32(data=0))

        current_path = self.custom_path_from_msg(goal_handle.request.path)
        self.controller.update_path(current_path)

        self.publish_reference_path()

        if hasattr(self.controller, "previous_input_array"):
            self.controller.previous_input_array = np.zeros((2, self.controller.horizon_length))
        self.controller.compute_distance_to_goal(self.state, 0)
        self.last_distance_to_goal = self.controller.linear_distance_to_goal
        self.controller.next_path_idx = 0

        self.get_logger().info(f"Initial state: {self.state}")
        self.get_logger().info(f"Ref path: {self.controller.path.poses}")
        self.get_logger().info(f"Distance to goal: {self.controller.linear_distance_to_goal} m.")

        while not self.controller.goal_reached():

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn("Goal canceled! Stopping robot.")
                self.stop_robot()
                self.clear_paths()
                return FollowPath.Result()

            command_vector = self.compute_next_command()
            self.publish_command(command_vector)
            self.publish_optimal_path()
            self.publish_target_path()
            self.publish_sampled_trajectories()
            self.print_debug()

            if (
                self.controller.next_path_idx >= self.controller.path.n_poses - 1
                and self.controller.linear_distance_to_goal > self.last_distance_to_goal
            ):
                break

            self.last_distance_to_goal = self.controller.linear_distance_to_goal
            self.rate.sleep()

        self.get_logger().info("SUCCESS")
        goal_handle.succeed()
        self.stop_robot()
        self.clear_paths()
        return FollowPath.Result(result_status=UInt32(data=1))

    def cancel_callback(self, goal):

        self.get_logger().info("Received cancel request.")
        return CancelResponse.ACCEPT

    def custom_path_from_msg(self, path_msg):

        poses = []
        for pose in path_msg.poses:
            x, y, z = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
            quat = pose.pose.orientation
            roll, pitch, yaw = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler("xyz")
            poses.append([x, y, z, roll, pitch, yaw])

        return CustomPath(np.array(poses))

    def compute_next_command(self):

        with self.state_mutex:
            if isinstance(self.controller, MPPI):
                # MPPI re-samples/re-optimizes from scratch every call (no fixed-horizon
                # command queue to step through), so always recompute on the latest state.
                command_vector = self.controller.compute_command_vector(self.state)
                self.last_compute_time = self.get_clock().now().nanoseconds * 1e-9
            elif self.last_compute_time < self.last_tf_time:
                command_vector = self.controller.compute_command_vector(self.state)
                self.last_compute_time = self.get_clock().now().nanoseconds * 1e-9
            elif self.last_compute_time > 0.0:
                command_vector, id = self.controller.get_next_command()
            else:
                self.get_logger().warn("No control sequence computed, using zero command.")
                command_vector = np.zeros(2)

        return command_vector

    def publish_command(self, command_vector):
        self.cmd_vel_msg.header.frame_id = self.robot_frame
        self.cmd_vel_msg.header.stamp = self.get_clock().now().to_msg()
        self.cmd_vel_msg.twist.linear.x = command_vector[0]
        self.cmd_vel_msg.twist.angular.z = command_vector[1]
        self.command_pub.publish(self.cmd_vel_msg)

    def publish_optimal_path(self):

        optim_path_msg = Path()
        optim_path_msg.header.stamp = self.get_clock().now().to_msg()
        optim_path_msg.header.frame_id = self.map_frame
        optim_path_msg.poses = []

        if isinstance(self.controller, MPPI):
            # optim_trajectory: [horizon_length, 3] ([x, y, yaw] per row)
            optim_trajectory = np.asarray(self.controller.optim_trajectory)
            for k in range(0, self.controller.horizon_length):
                pose = self.planar_state_to_pose_msg(optim_trajectory[k, :])
                pose.pose.position.z = 0.1
                optim_path_msg.poses.append(pose)
        else:
            # optim_trajectory_array: [3, horizon_length] ([x, y, yaw] per column)
            for k in range(0, self.controller.horizon_length):
                pose = self.planar_state_to_pose_msg(self.controller.optim_trajectory_array[:, k])
                pose.pose.position.z = 0.1
                optim_path_msg.poses.append(pose)

        self.optimal_path_pub.publish(optim_path_msg)

    def publish_target_path(self):

        target_path_msg = Path()
        target_path_msg.header.stamp = self.get_clock().now().to_msg()
        target_path_msg.header.frame_id = self.map_frame
        target_path_msg.poses = []

        if isinstance(self.controller, MPPI):
            # target_trajectory: [horizon_length, 3] ([x, y, yaw] per row)
            target_trajectory = np.asarray(self.controller.target_trajectory)
            for k in range(0, self.controller.horizon_length):
                pose = self.planar_state_to_pose_msg(target_trajectory[k, :])
                pose.pose.position.z = 0.05
                target_path_msg.poses.append(pose)
        else:
            # target_trajectory: [3, horizon_length] ([x, y, yaw] per column)
            for k in range(0, self.controller.horizon_length):
                pose = self.planar_state_to_pose_msg(self.controller.target_trajectory[:, k])
                pose.pose.position.z = 0.05
                target_path_msg.poses.append(pose)

        self.target_path_pub.publish(target_path_msg)

    def publish_sampled_trajectories(self):
        # Mirrors the gray sampled-rollout cloud drawn in mppi.py's own debug plot.
        if not isinstance(self.controller, MPPI):
            return

        sampled_states = np.asarray(self.controller.sampled_states)  # [n_samples, horizon_length, 3]
        n_samples = sampled_states.shape[0]
        n_to_plot = max(1, int(n_samples * self.sampled_trajectories_plot_fraction))
        plot_idx = np.random.choice(n_samples, n_to_plot, replace=False)

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.map_frame
        marker.ns = "sampled_trajectories"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 0.2

        points = []
        for idx in plot_idx:
            trajectory = sampled_states[idx]  # [horizon_length, 3]
            for k in range(trajectory.shape[0] - 1):
                points.append(Point(x=float(trajectory[k, 0]), y=float(trajectory[k, 1]), z=0.0))
                points.append(Point(x=float(trajectory[k + 1, 0]), y=float(trajectory[k + 1, 1]), z=0.0))
        marker.points = points

        marker_array_msg = MarkerArray()
        marker_array_msg.markers.append(marker)
        self.sampled_trajectories_pub.publish(marker_array_msg)

    def publish_reference_path(self):

        ref_path_msg = Path()
        ref_path_msg.header.stamp = self.get_clock().now().to_msg()
        ref_path_msg.header.frame_id = self.map_frame
        ref_path_msg.poses = []

        for k in range(0, self.controller.path.n_poses):
            planar_state = [
                self.controller.path.poses[k, 0],
                self.controller.path.poses[k, 1],
                self.controller.path.angles[k],
            ]
            pose = self.planar_state_to_pose_msg(planar_state)
            ref_path_msg.poses.append(pose)

        self.reference_path_pub.publish(ref_path_msg)

    def planar_state_to_pose_msg(self, planar_state):

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.map_frame
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position = Point(x=float(planar_state[0]), y=float(planar_state[1]), z=0.0)
        x, y, z, w = R.from_euler("xyz", [0.0, 0.0, planar_state[2]]).as_quat()  # type: ignore
        pose_msg.pose.orientation = Quaternion(x=x, y=y, z=z, w=w)
        return pose_msg

    def publish_distance_to_goal(self):

        self.goal_linear_distance_pub.publish(Float32(data=self.controller.linear_distance_to_goal))
        self.goal_angular_distance_pub.publish(Float32(data=self.controller.angular_distance_to_goal))

    def stop_robot(self):
        self.cmd_vel_msg = TwistStamped()
        self.cmd_vel_msg.header.stamp = self.get_clock().now().to_msg()
        self.command_pub.publish(self.cmd_vel_msg)

    def clear_paths(self):

        empty_path_msg = Path()
        self.reference_path_pub.publish(empty_path_msg)
        self.target_path_pub.publish(empty_path_msg)
        self.optimal_path_pub.publish(empty_path_msg)

        delete_all_marker = Marker()
        delete_all_marker.ns = "sampled_trajectories"
        delete_all_marker.id = 0
        delete_all_marker.action = Marker.DELETEALL
        self.sampled_trajectories_pub.publish(MarkerArray(markers=[delete_all_marker]))

    def print_debug(self):

        self.get_logger().debug(f"Planar state : {self.controller.planar_state}")
        self.get_logger().debug(f"Linear distance to goal: {self.controller.linear_distance_to_goal}")
        self.get_logger().debug(f"Angular distance to goal: {self.controller.angular_distance_to_goal}")

        if isinstance(self.controller, MPPI):
            self.get_logger().debug(f"Next command (wheel vel, left/right): {self.controller.a_opt[0]}")
            self.get_logger().debug(f"Target trajectory: {self.controller.target_trajectory}")
        else:
            self.get_logger().debug(
                f"Next command : (Left) {self.controller.optimal_left}, (Right) {self.controller.optimal_right}"
            )
            self.get_logger().debug(f"Target path: {self.controller.target_trajectory.T}")
            for j in range(0, self.controller.horizon_length):
                self.get_logger().debug(f"optimal_left_{j} {self.controller.optim_solution_array[j]}")
                self.get_logger().debug(
                    f"optimal_right_{j} {self.controller.optim_solution_array[j + self.controller.horizon_length]}"
                )
            self.get_logger().debug(f"Debug indicator: {str(self.controller.debug_indicator)}")


def main(args=None):

    rclpy.init(args=args)

    try:
        controller_node = ControllerNode()
        executor = MultiThreadedExecutor()
        executor.add_node(controller_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            controller_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
