import std_msgs.msg
from norlabcontrollib.controllers.controller_factory import ControllerFactory
from norlabcontrollib.path.path import Path
import numpy as np
from multiprocessing import Lock
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as Ros2Path

from norlab_controllers_msgs.msg import PathSequence, DirectionalPath
from norlab_controllers_msgs.action import FollowPath


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')
        self.declare_parameter('controller_config')
        controller_config_path = self.get_parameter('controller_config').get_parameter_value().string_value
        self.declare_parameter('rotation_controller_config')
        rotation_controller_config_path = self.get_parameter('rotation_controller_config').get_parameter_value().string_value

        self.controller_factory = ControllerFactory()
        self.controller = self.controller_factory.load_parameters_from_yaml(controller_config_path)
        self.rotation_controller = self.controller_factory.load_parameters_from_yaml(rotation_controller_config_path)

        self.state = np.zeros(6) # [x, y, z, roll, pitch, yaw]
        self.velocity = np.zeros(6) # [vx, vy, vz, v_roll, v_pitch, v_yaw]
        self.state_velocity_mutex = Lock()

        self.cmd_publisher_ = self.create_publisher(Twist, 'cmd_vel_out', 100)
        self.cmd_vel_msg = Twist()
        self.optim_path_publisher_ = self.create_publisher(Ros2Path, 'optimal_path', 100)
        self.optim_path_msg = Ros2Path()
        self.optim_path_msg.header.frame_id = "map"
        self.target_path_publisher_ = self.create_publisher(Ros2Path, 'target_path', 100)
        self.target_path_msg = Ros2Path()
        self.target_path_msg.header.frame_id = "map"
        self.ref_path_publisher_ = self.create_publisher(Ros2Path, 'ref_path', 100)
        self.ref_path_msg = Ros2Path()
        self.ref_path_msg.header.frame_id = "map"

        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom_in',
            self.odometry_callback,
            10)

        self._action_server = ActionServer(
            self,
            FollowPath,
            'follow_path',
            self.follow_path_callback
        )

        self.rate = self.create_rate(self.controller.rate)

        self.waiting_for_path = True
        self.loading_path = False
        self.executing_path = False
        self.get_logger().info('angular_velocity_gain ='+str(self.controller.angular_velocity_gain))
        self.get_logger().info('state_cost_translational ='+str(self.controller.state_cost_translational))
        self.get_logger().info('state_cost_rotational ='+str(self.controller.state_cost_rotational))
        self.get_logger().info('input_cost_wheel ='+str(self.controller.input_cost_wheel))

    def quaternion_to_euler(self, w, x, y, z):
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x ** 2 + y ** 2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.where(np.abs(sinp) >= 1,
                         np.sign(sinp) * np.pi / 2,
                         np.arcsin(sinp))

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y ** 2 + z ** 2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch, yaw])

    def odometry_callback(self, message):
        with self.state_velocity_mutex:
            self.state[0] = message.pose.pose.position.x
            self.state[1] = message.pose.pose.position.y
            self.state[2] = message.pose.pose.position.z
            self.state[3:] = self.quaternion_to_euler(message.pose.pose.orientation.w,
                                                      message.pose.pose.orientation.x,
                                                      message.pose.pose.orientation.y,
                                                      message.pose.pose.orientation.z)
            self.velocity[0] = message.twist.twist.linear.x
            self.velocity[1] = message.twist.twist.linear.y
            self.velocity[2] = message.twist.twist.linear.z
            self.velocity[3] = message.twist.twist.angular.x
            self.velocity[4] = message.twist.twist.angular.y
            self.velocity[5] = message.twist.twist.angular.z

    def command_array_to_twist_msg(self, command_array):
        self.cmd_vel_msg.linear.x = command_array[0]
        self.cmd_vel_msg.linear.y = 0.0
        self.cmd_vel_msg.linear.z = 0.0
        self.cmd_vel_msg.angular.x = 0.0
        self.cmd_vel_msg.angular.y = 0.0
        self.cmd_vel_msg.angular.z = command_array[1]

    def compute_then_publish_command(self):
        with self.state_velocity_mutex:
            command_vector = self.controller.compute_command_vector(self.state)
            self.command_array_to_twist_msg(command_vector)
            self.cmd_publisher_.publish(self.cmd_vel_msg)

    def compute_then_publish_rotation_command(self):
        with self.state_velocity_mutex:
            command_vector = self.rotation_controller.compute_command_vector(self.state)
            self.command_array_to_twist_msg(command_vector)
            self.cmd_publisher_.publish(self.cmd_vel_msg)

    def publish_optimal_path(self):
        self.optim_path_msg.header.stamp = self.get_clock().now().to_msg()
        self.optim_path_msg = Ros2Path()
        self.optim_path_msg.header.frame_id = "map"
        for k in range(0, self.controller.horizon_length):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = self.controller.optim_trajectory_array[0, k]
            pose.pose.position.y = self.controller.optim_trajectory_array[1, k]
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            self.optim_path_msg.poses.append(pose)
        self.optim_path_publisher_.publish(self.optim_path_msg)

    def publish_target_path(self):
        self.target_path_msg.header.stamp = self.get_clock().now().to_msg()
        self.target_path_msg = Ros2Path()
        self.target_path_msg.header.frame_id = "map"
        for k in range(0, self.controller.horizon_length):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = self.controller.target_trajectory[0, k]
            pose.pose.position.y = self.controller.target_trajectory[1, k]
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            self.target_path_msg.poses.append(pose)
        self.target_path_publisher_.publish(self.target_path_msg)

    def publish_reference_path(self):
        self.ref_path_msg.header.stamp = self.get_clock().now().to_msg()
        self.ref_path_msg = Ros2Path()
        self.ref_path_msg.header.frame_id = "map"
        for k in range(0, self.controller.path.n_poses):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = self.controller.path.poses[k, 0]
            pose.pose.position.y = self.controller.path.poses[k, 1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            self.ref_path_msg.poses.append(pose)
        self.ref_path_publisher_.publish(self.ref_path_msg)


    def follow_path_callback(self, path_goal_handle):
        ## Importing all goal paths
        self.get_logger().info("Importing goal paths...")
        goal_paths = path_goal_handle.request.path.paths
        self.path_goal_handle = path_goal_handle
        self.goal_paths_list = []
        self.goal_paths_directions_list = []
        for current_path in goal_paths:
            current_path_length = len(current_path.poses)
            current_path_array = np.zeros((current_path_length, 6))
            for i in range(0, current_path_length):
                current_path_array[i, 0] = current_path.poses[i].pose.position.x
                current_path_array[i, 1] = current_path.poses[i].pose.position.y
                current_path_array[i, 2] = current_path.poses[i].pose.position.z
                current_path_array[i, 3:] = self.quaternion_to_euler(current_path.poses[i].pose.orientation.w,
                                                                     current_path.poses[i].pose.orientation.x,
                                                                     current_path.poses[i].pose.orientation.y,
                                                                     current_path.poses[i].pose.orientation.z)
            current_path_object = Path(current_path_array)
            current_path_object.going_forward = current_path.forward
            current_path_object.compute_metrics(self.controller.path_look_ahead_distance)
            self.goal_paths_list.append(current_path_object)
        self.number_of_goal_paths = len(self.goal_paths_list)
        self.get_logger().info("Path import done, proceeding to executing " + str(self.number_of_goal_paths) + " path(s)...")

        ## execute paths one by one
        for i in range(0, self.number_of_goal_paths):
            # load all goal paths in sequence
            self.get_logger().info("Executing path " + str(i + 1) + " of " + str(self.number_of_goal_paths))
            self.controller.update_path(self.goal_paths_list[i])
            self.rotation_controller.update_path(self.goal_paths_list[i])
            self.publish_reference_path()
            self.controller.previous_input_array = np.zeros((2, self.controller.horizon_length))
            # while loop to repeat a single goal path
            if i > 0:
                while self.rotation_controller.angular_distance_to_goal >= self.rotation_controller.goal_tolerance:
                    self.compute_then_publish_rotation_command()
                    self.rate.sleep()
            self.last_distance_to_goal = 1000
            self.controller.compute_distance_to_goal(self.state, 0)
            self.controller.last_path_pose_id = 0
            # self.get_logger().info('reftraj_x0' + str(self.controller.path.poses[0,0]))
            # self.get_logger().info('reftraj_y0' + str(self.controller.path.poses[0,1]))
            for j in range(0, self.controller.path.n_poses):
                self.get_logger().info('ref_traj_x_' + str(j) + ' ' + str(self.controller.path.poses[j, 0]))
                self.get_logger().info('ref_traj_y_' + str(j) + ' ' + str(self.controller.path.poses[j, 1]))
            while self.controller.euclidean_distance_to_goal >= self.controller.goal_tolerance:
                self.compute_then_publish_command()
                self.publish_optimal_path()
                self.publish_target_path()
                # self.get_logger().info('optimal left : ' + str(self.controller.optimal_left))
                # self.get_logger().info('optimal right : ' + str(self.controller.optimal_right))
                # self.get_logger().info('controller_x : ' + str(self.controller.planar_state[0]))
                # self.get_logger().info('controller_y : ' + str(self.controller.planar_state[1]))
                # self.get_logger().info('controller_yaw : ' + str(self.controller.planar_state[2]))
                # for j in range(0, self.controller.horizon_length):
                #     self.get_logger().info('target_traj_x_' + str(j) + ' ' + str(self.controller.target_trajectory[0, j]))
                #     self.get_logger().info('target_traj_y_' + str(j) + ' ' + str(self.controller.target_trajectory[1, j]))
                    # self.get_logger().info('optimal_left_' + str(j) + ' ' + str(self.controller.optim_solution_array[j]))
                    # self.get_logger().info('optimal_right_' + str(j) + ' ' + str(self.controller.optim_solution_array[j + self.controller.horizon_length]))
                # self.get_logger().info('Path Curvature : ' + str(self.controller.path_curvature))
                # self.get_logger().info('look ahead distance counter : ' + str(self.controller.look_ahead_distance))
                # self.get_logger().info('Distance_to_goal : ' + str(self.controller.distance_to_goal))
                # self.get_logger().info('Euclidean Distance_to_goal : ' + str(self.controller.euclidean_distance_to_goal))
                if self.controller.orthogonal_projection_id >= self.controller.path.n_poses-1:
                    if self.controller.euclidean_distance_to_goal > self.last_distance_to_goal:
                        break
                    else:
                        self.last_distance_to_goal = self.controller.euclidean_distance_to_goal
                self.rate.sleep()
            # self.controller.last_path_pose_id = 0

        self.cmd_vel_msg = Twist()
        self.cmd_publisher_.publish(self.cmd_vel_msg)

        self.get_logger().info("SUCCESS")

        ## return completed path to action client
        path_goal_handle.succeed()
        paths_result = FollowPath.Result()
        result_status = std_msgs.msg.UInt32()
        result_status.data = 1  # 1 for success
        paths_result.result_status = result_status
        return paths_result

    def follow_path_spinner(self):
        if self.executing_path:
            self.compute_then_publish_command()
            if self.controller.distance_to_goal <= self.controller.goal_tolerance:
                self.path_id += 1
                if self.path_id >= self.number_of_goal_paths:
                    self.get_logger().info("SUCCESS")
                    ## return completed path to action client
                    self.path_goal_handle.set_succeeded()
                    paths_result = FollowPath.Result()
                    result_status = std_msgs.msg.UInt32()
                    result_status.data = 1  # 1 for success
                    paths_result.result_status = result_status
                    return paths_result
                    self.executing_path = False
                    return None
                self.get_logger().info("Executing path " + str(self.path_id + 1) + " of " + str(self.number_of_goal_paths))
                self.controller.update_path(self.goal_paths_list[self.path_id])

            else:
                return None


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


if __name__ == '__main__':
    main()
