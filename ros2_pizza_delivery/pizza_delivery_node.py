import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point, Pose, Quaternion

class PizzaDeliveryNode(Node):
    def __init__(self):
        super().__init__('pizza_delivery_node')

        self.waypoints = [
            Pose(
                position=Point(x=1.028060, y=1.737860, z=0.149815),
            ),
            Pose(
                position=Point(x=0.676417, y=0.026173, z=0.149815),
            ),
            Pose(
                position=Point(x=0.620023, y=1.416080, z=0.149815)
            )
        ]

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

        self.current_waypoint_index = 0
        self.subscriber = self.create_subscription()
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
            self.client.send_goal_async(goal_msg).add_done_callback(self.navigation_callback)
        else:
            self.get_logger().info('All waypoints reached.')

    def navigation_callback(self, future):
        response = future.result()
        if response.accepted:
            self.get_logger().info('Waypoint reached.')
            self.current_waypoint_index += 1
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