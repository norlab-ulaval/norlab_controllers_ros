import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel_out', 100)
        self.subscription = self.create_subscription(
            Odometry,
            'odom_in',
            self.odometry_callback,
            10)
        self.pose = np.zeros(6) # [x, y, z, roll, pitch, yaw]
        self.velocity = np.zeros(6) # [vx, vy, vz, v_roll, v_pitch, v_yaw]
        self.subscription

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

def main(args=None):
    rclpy.init(args=args)

    controller_node = ControllerNode()

    rclpy.spin(controller_node)

    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()