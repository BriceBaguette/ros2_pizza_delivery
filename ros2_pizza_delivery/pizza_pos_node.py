"""
This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
   /aruco_markers
   /aruco_poses
   /odom

Published Topics:
    

Parameters:


Author: Brice Baguette

"""

import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
from ros2_aruco import transformations

from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
from ros2_pizza_delivery_interfaces.msg import PizzaPose

class PizzaPosNode(rclpy.node.Node):

    def __init__(self, nb_marker):
        super().__init__('pizza_pos_node')
        self.marker_sub = self.create_subscription(ArucoMarkers,
                                                       '/aruco_markers',
                                                       self.callback,
                                                       10)
        self.publisher = self.create_publisher(
            PizzaPose,
            'pizza_pos',
            10
        )
        self.pizza_pos = dict()
        self.nb_marker = nb_marker
    def callback(self, data):
        for i in range(len(data.marker_ids)):
            if(data.marker_ids[i] not in self.pizza_pos):
                self.get_logger().info("%s" %data)
                self.pizza_pos[data.marker_ids[i]] = data.poses
                if(len(self.pizza_pos) == self.nb_marker):
                    self.publisher.publish_msg(self.pizza_pos)

def main(args):
    rclpy.init(args)
    node = PizzaPosNode(args[1])
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
