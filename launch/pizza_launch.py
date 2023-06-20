from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Create an instance of LaunchDescription
    ld = LaunchDescription()

    path_planner = Node(
    package='ros2_pizza_delivery',
    executable='path_planner_node',
    name='path_planner_node',
    output='screen'
)

    pizza_delivery = Node(
    package='ros2_pizza_delivery',
    executable='pizza_delivery_node',
    name='pizza_delivery_node',
    output='screen'
)

    ld.add_action(path_planner)
    ld.add_action(pizza_delivery)

    return ld