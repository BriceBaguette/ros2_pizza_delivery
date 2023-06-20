from numpy import size
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped
from std_msgs.msg import Header

import numpy as np
import tf2_ros
# from ros2_pizza_interfaces.msg import PizzaPose
import yaml
import math

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')
        self.occupancy_grid = OccupancyGrid
        self.matrix_map = []
        self.waypoints = []
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.load_map,
            10
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            Path,
            'pizza_path',
            10
        )
        #self.subscription = self.create_subscription(
        #    PizzaPose,
        #    'pizza_pose',
        #   self.pose_callback(),
        #   10
        #)

    def pose_callback(self):
        success = False
        while not success:
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map',  # Target frame (map frame in this example)
                    'base_footprint',  # Source frame (base_link frame in this example)
                    rclpy.time.Time().to_msg(),  # Use latest available transform
                    timeout=rclpy.duration.Duration(seconds=1)
                )
                # Access the pose information
                pose = transform.transform
                self.waypoints = [Pose(position=Point(x = pose.translation.x, y= pose.translation.y, z= pose.translation.z), orientation=pose.rotation)]
                """
                quaternion1 = self.euler_to_quaternion(0.000400, 1.570000, 1.570396)
                quaternion2 = self.euler_to_quaternion(0.005486,1.570000,2.618564)
                self.waypoints.extend([Pose(
                    position=Point(x=0.676417, y=0.026173, z=0.149815),
                    orientation=quaternion1
                ),
                
                Pose(
                    position=Point(x=1.028060, y=1.737860, z=0.149815),
                    orientation=Quaternion( x = 0.000000,
                                            y = 1.570000,
                                            z = 0.000000,
                                            w = 0.0)
                ),

                Pose(
                position=Point(x=0.620023, y=1.416080, z=0.149815),
                orientation=quaternion2)
                ])
                """
                """
                for i in range(self.nb_marker):
                    transform = self.tf_buffer.lookup_transform('map','aruco_'+str(i), rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
                    print(transform)
                """
                success = True
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                # Handle exceptions
                self.get_logger().warning('Failed to retrieve TurtleBot3 pose')

    def load_map(self, msg):
        self.occupancy_grid = msg
        map_data = msg.data
        map_width = msg.info.width
        map_height = msg.info.height

        matrix = [[0 for _ in range(map_width)] for _ in range(map_height)]

        for y in range(map_height):
            for x in range(map_width):
                index = y * map_width + x
                map_value = map_data[index]
                if map_value == 100:
                    matrix[y][x] = 1  # Represents a wall or obstacle
        self.matrix_map = matrix
        if len(self.waypoints)==0:
            self.pose_callback()
        self.get_path()

    def get_path(self):
            matrix_representation = [self.pose_to_occupancy_grid(pose) for pose in self.waypoints]
            path = self.find_best_order(matrix_representation)
            converted_path = [self.occupancy_grid_to_pose(pose[0], pose[1], pose[2], pose[3]) for pose in path]
            self.publish_path(converted_path)

    def calculate_distance(self, point1, point2):
        """Calculate the Euclidean distance between two points."""
        dx = point2[0] - point1[0]
        dy = point2[1] - point1[1]
        distance = math.sqrt(dx**2 + dy**2)
        return distance

    def find_best_order(self, waypoints):
        """Find the best order to visit the list of waypoints."""
        num_waypoints = len(waypoints)
        best_order = []

        # Start with the first waypoint as the initial current position
        current_position = waypoints[0]
        best_order.append(current_position)

        # Continue until all waypoints have been visited
        while len(best_order) < num_waypoints:
            closest_distance = float('inf')
            closest_point = None

            # Find the closest unvisited waypoint
            for waypoint in waypoints:
                if waypoint not in best_order:
                    distance = self.calculate_distance(current_position, waypoint)
                    if distance < closest_distance:
                        closest_distance = distance
                        closest_point = waypoint

            # Update the current position and add the closest waypoint to the best order
            current_position = closest_point
            best_order.append(current_position)

        return best_order

    def publish_path(self, path):
        del path[0]
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.frame_id = 'map'

        for position in path:
            pose = PoseStamped()
            pose.header = Header()
            pose.header.frame_id = 'map'
            pose.pose = self.get_pose_in_front(position, 0.2)

            path_msg.poses.append(pose)
        self.publisher.publish(path_msg)
    
    def occupancy_grid_to_pose(self, grid_x, grid_y, z, orientation):
        resolution = self.occupancy_grid.info.resolution
        origin_x = self.occupancy_grid.info.origin.position.x
        origin_y = self.occupancy_grid.info.origin.position.y

        pose_x = origin_x + (grid_x * resolution)
        pose_y = origin_y + (grid_y * resolution)

        pose = Pose()
        pose.position.x = pose_x
        pose.position.y = pose_y
        pose.position.z = z
        pose.orientation = orientation

        return pose
    
    def pose_to_occupancy_grid(self, pose):
        resolution = self.occupancy_grid.info.resolution
        origin_x = self.occupancy_grid.info.origin.position.x
        origin_y = self.occupancy_grid.info.origin.position.y

        pose_x = pose.position.x
        pose_y = pose.position.y

        grid_x = int((pose_x - origin_x) / resolution)
        grid_y = int((pose_y - origin_y) / resolution)

        return grid_x, grid_y, pose.position.z, pose.orientation

    def get_pose_in_front(self, pose, distance):
        # Extract position and orientation from the given pose
        position = pose.position
        orientation = self.quaternion_to_euler(pose.orientation)
        print(orientation)

        # Calculate the displacement in the x and y directions based on the orientation
        dx = math.cos(orientation[2]) * distance
        dy = math.sin(orientation[2]) * distance

        # Calculate the new position in front of the original pose
        new_position = Point()
        new_position.x = position.x + dx
        new_position.y = position.y + dy
        new_position.z = position.z

        # Create a new Pose object with the calculated position and the opposite orientation
        new_pose = Pose()
        new_pose.position = new_position
        new_pose.orientation = self.euler_to_quaternion(0,0,(math.pi + orientation[0]))
        return new_pose

    def convert_pose_list(self, pose_list, distance):
        converted_list = []

        for pose in pose_list:
            new_pose = self.get_pose_in_front(pose, distance)
            converted_list.append(new_pose)

        return converted_list



    def quaternion_to_euler(self,quaternion):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw).

        :param quaternion: geometry_msgs.msg.Quaternion representing the quaternion.
        :return: Tuple of Euler angles (roll, pitch, yaw) in radians.
        """
        q = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        roll = math.atan2(2 * (q[3] * q[0] + q[1] * q[2]), 1 - 2 * (q[0]**2 + q[1]**2))
        pitch = math.asin(2 * (q[3] * q[1] - q[2] * q[0]))
        yaw = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1]**2 + q[2]**2))
        return roll, pitch, yaw


    def euler_to_quaternion(self,roll, pitch, yaw):
        """
        Convert Euler angles (roll, pitch, yaw) to quaternion.

        :param roll: Roll angle in radians.
        :param pitch: Pitch angle in radians.
        :param yaw: Yaw angle in radians.
        :return: geometry_msgs.msg.Quaternion representing the quaternion.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        quaternion = Quaternion()
        quaternion.x = sr * cp * cy - cr * sp * sy
        quaternion.y = cr * sp * cy + sr * cp * sy
        quaternion.z = cr * cp * sy - sr * sp * cy
        quaternion.w = cr * cp * cy + sr * sp * sy
        return quaternion



def main(args=None):
    rclpy.init(args=args)
    path_planner_node = PathPlannerNode()
    rclpy.spin(path_planner_node)
    path_planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
