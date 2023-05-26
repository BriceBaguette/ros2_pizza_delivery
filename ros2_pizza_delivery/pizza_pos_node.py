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
from tf2_ros import Buffer, TransformListener
from ros2_pizza_interfaces.msg import PizzaPose

class PizzaPosNode(rclpy.node.Node):

    def __init__(self, nb_marker):
        super().__init__('pizza_pos_node')
        self.publisher = self.create_publisher(
            PizzaPose,
            'pizza_pos',
            10
        )
        self.tf_buffer = Buffer()
        self.pizza_pos = dict()
        self.nb_marker = nb_marker
        self.callback()

    def callback(self):
        for i in range(self.nb_marker):
            self.get_logger(self.tf_buffer.lookupTransform('map','aruco_'+i, rclpy.time.Time()))

def main(args):
    rclpy.init(args)
    node = PizzaPosNode(args[1])
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
