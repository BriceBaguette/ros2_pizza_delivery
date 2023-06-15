import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Pose, Quaternion
import math
import time

class PizzaDeliveryNode(Node):
    def __init__(self):
        super().__init__('pizza_delivery_node')

        self.waypoints = []
        self.callback_processed = False

        self.publisher = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10
        )


        self.client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        while not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Action server not available, waiting...')

        self.subscriber = self.create_subscription(
            Path,
            'pizza_path',
            self.start_navigate_callback,
            10
        )
    def start_navigate_callback(self, msg):
        if self.callback_processed:
            return
        self.callback_processed = True
        for pose in msg.poses:
            self.waypoints.append(pose.pose)
        self.current_waypoint_index = 0
        self.navigate_to_next_waypoint()

    def navigate_to_next_waypoint(self):
        if self.current_waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_waypoint_index]
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose = waypoint

            self.publisher.publish(pose_msg)

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose_msg
            print(goal_msg.pose)
            goal_handle_future = self.client.send_goal_async(goal_msg)
            goal_handle_future.add_done_callback(self.navigation_goal_sent_callback)
        else:
            self.get_logger().info('All waypoints reached.')

    def navigation_goal_sent_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected.')
            return

        self.get_logger().info('Navigation goal accepted.')
        goal_handle.get_result_async().add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        result = future.result()
        print(result)
        if result.status == 4:
            self.get_logger().info('Waypoint reached.')
            self.current_waypoint_index += 1
            time.sleep(1)
            self.navigate_to_next_waypoint()
        else:
            self.get_logger().info('Navigation failed.')

def main():
    rclpy.init()
    pizza_delivery_node = PizzaDeliveryNode()
    rclpy.spin(pizza_delivery_node)
    print("oups")
    pizza_delivery_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    print("hi")
    main()