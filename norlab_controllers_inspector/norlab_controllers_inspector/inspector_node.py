# Author : Cyril Goffin
# Last modified : 30/05/2023

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
import numpy as np
import pandas as pd
from norlab_controllers_msgs.srv import ExportData


class ControllerSubscriber(Node):

    def __init__(self):
        super().__init__('controller_subscriber')

        self.loc_x = []
        self.loc_y = []

        self.ref_path_x = []
        self.ref_path_y = []

        self.loc_subscription = self.create_subscription(Odometry, 'icp_odom', self.loc_callback, 10)
        self.loc_subscription

        self.ref_path_subscription = self.create_subscription(Path, 'ref_path', self.ref_path_callback, 10)
        self.ref_path_subscription

        self.srv = self.create_service(ExportData, 'export_data', self.export_data_callback)

    def loc_callback(self, msg):
        current_loc_x = msg.pose.pose.position.x
        current_loc_y = msg.pose.pose.position.y
        #self.get_logger().info('Current loc: (%s, %s)' % (current_loc_x, current_loc_y))

        self.loc_x.append(current_loc_x)
        self.loc_y.append(current_loc_y)

    def ref_path_callback(self, msg):
        for pose in msg.poses:
            self.get_logger().info('Current ref path: (%s, %s)' % (pose.pose.position.x, pose.pose.position.y))

            self.ref_path_x.append(pose.pose.position.x)
            self.ref_path_y.append(pose.pose.position.y)

    def export_data_callback(self, request, response):
        loc_data = np.array([self.loc_x, self.loc_y]).T
        ref_path_data = np.array([self.ref_path_x, self.ref_path_y]).T

        loc_df = pd.DataFrame(loc_data, columns = ['loc_x','loc_y'])
        ref_path_df = pd.DataFrame(ref_path_data, columns = ['ref_path_x','ref_path_y'])

        loc_df.to_pickle('%s/loc.pkl' % (request.export_path.data))
        ref_path_df.to_pickle('%s/ref_path.pkl' % (request.export_path.data))

        return response


def main(args=None):
    rclpy.init(args=args)

    controller_subscriber = ControllerSubscriber()

    rclpy.spin(controller_subscriber)

    controller_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
