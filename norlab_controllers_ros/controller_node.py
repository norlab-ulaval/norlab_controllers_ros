from norlabcontrollib.controllers.controller_factory import ControllerFactory
from norlabcontrollib.path.path import Path

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from norlab_controllers_msgs.msg import PathSequence, DirectionalPath
from norlab_controllers_msgs.action import FollowPath

#TODO: Get a wiln working setup to send action goals to this node to iterate more quickly

class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')
        self.cmd_publisher_ = self.create_publisher(Twist, 'cmd_vel_out', 100)
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

        self.declare_parameter('controller_config')
        # controller_config_path = self.get_parameter('controller_config').get_parameter_value().string_value
        controller_config_path = "/home/dominic/ros2_ws/install/norlab_controllers_ros/share/norlab_controllers_ros/warthog-differential-orthexp.yaml"

        self.controller_factory = ControllerFactory()
        self.controller = self.controller_factory.load_parameters_from_yaml(controller_config_path)

        self.get_logger().info(str(self.controller.path_look_ahead_distance))

        self.state = np.zeros(6) # [x, y, z, roll, pitch, yaw]
        self.velocity = np.zeros(6) # [vx, vy, vz, v_roll, v_pitch, v_yaw]
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

    def follow_path_callback(self, path_goal_handle):
        ## Importing all goal paths
        self.get_logger().info("Importing goal paths...")
        goal_paths = path_goal_handle.request.path.paths
        self.goal_paths_list = []
        for current_path in goal_paths:
            current_path_length = len(current_path.poses)
            current_path_array = np.zeros((current_path_length, 6))
            for i in range(0, current_path_length):
                current_path_array[i, 0] = current_path.poses[i].pose.position.x
                current_path_array[i, 1] = current_path.poses[i].pose.position.y
                current_path_array[i, 2] = current_path.poses[i].pose.position.z
                current_orientation_euler = self.quaternion_to_euler(current_path.poses[i].pose.orientation.w,
                                                                     current_path.poses[i].pose.orientation.x,
                                                                     current_path.poses[i].pose.orientation.y,
                                                                     current_path.poses[i].pose.orientation.z)
                current_path_array[i, 3:] = current_orientation_euler
                current_path_object = Path(current_path_array)
                current_path_object.compute_metrics()
            self.goal_paths_list.append(current_path_object)
        number_of_goal_paths = len(self.goal_paths_list)
        self.get_logger().info("Path import done, proceeding to executing " + str(number_of_goal_paths) + " path(s)...")

        ## execute paths one by one
        for i in range(0, number_of_goal_paths):
            # load all goal paths in sequence
            self.get_logger().info("Executing path " + str(i+1) + " of " + str(number_of_goal_paths))
            self.controller.update_path(self.goal_paths_list[i])
            # while loop to repeat a single goal path
            while self.controller.distance_to_goal >= self.controller.goal_tolerance:
                self.controller.compute_command_vector(self.state)

        ## return completed path to action client
        paths_result = FollowPath.Result()
        paths_result.result_status = 1 # 1 for success
        return paths_result

def main(args=None):
    rclpy.init(args=args)

    controller_node = ControllerNode()

    rclpy.spin(controller_node)

    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()