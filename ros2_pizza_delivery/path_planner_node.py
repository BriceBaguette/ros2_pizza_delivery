import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from ros2_pizza_interfaces.msg import PizzaPose
import heapq


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )

        self.publisher = self.create_publisher(
            Path,
            'path',
            10
        )

        self.subscription = self.create_subscription(
            PizzaPose,
            'pizza_pose',
            self.pose_callback(),
            10
        )
        self.map = None
        self.waypoints = dict

    def pose_callback(self,msg):
        self.waypoints = msg.poses

    def map_callback(self, msg):
        self.map = msg

        # Once the map is received, start the path planning
        path = self.astar_path_planning(self.waypoints)
        self.publish_path(path)

    def astar_path_planning(self, waypoints):
        start = waypoints[0]
        goal = waypoints[-1]

        open_list = []
        closed_set = set()

        start_node = Node(start, 0, self.calculate_heuristic(start, goal))
        heapq.heappush(open_list, start_node)

        while open_list:
            current_node = heapq.heappop(open_list)

            if current_node.position == goal:
                return self.extract_path(current_node)

            closed_set.add(current_node.position)

            for neighbor in self.get_neighbors(current_node.position):
                if neighbor in closed_set:
                    continue

                neighbor_node = Node(neighbor, current_node.g_cost + self.calculate_distance(current_node, neighbor),
                                     self.calculate_heuristic(neighbor, goal), current_node)
                heapq.heappush(open_list, neighbor_node)

        return []

    def get_neighbors(self, position):
        neighbors = []
        map_width = self.map.info.width
        map_height = self.map.info.height

        # Define the threshold value for considering a cell as an obstacle
        obstacle_threshold = 50

        # Define the indices of the neighboring cells
        indices = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right

        for dx, dy in indices:
            nx = position[0] + dx
            ny = position[1] + dy

            # Check if the neighboring cell is within the map boundaries
            if 0 <= nx < map_width and 0 <= ny < map_height:
                # Check the occupancy value of the neighboring cell
                cell_index = nx + ny * map_width
                cell_value = self.map.data[cell_index]

                # Exclude the cell if it is marked as an obstacle
                if cell_value < obstacle_threshold:
                    neighbors.append((nx, ny))

        return neighbors


    def calculate_distance(self, node1, node2):
        return ((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2) ** 0.5

    def calculate_heuristic(self, node, goal):
        return self.calculate_distance(node, goal)

    def extract_path(self, goal_node):
        path = []
        current_node = goal_node

        while current_node:
            path.append(current_node.position)
            current_node = current_node.parent

        path.reverse()
        return path

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.frame_id = 'map'

        for position in path:
            pose = PoseStamped()
            pose.header = Header()
            pose.header.frame_id = 'map'
            pose.pose.position.x = position[0]
            pose.pose.position.y = position[1]
            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.publisher.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    path_planner_node = PathPlannerNode()
    rclpy.spin(path_planner_node)
    path_planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
