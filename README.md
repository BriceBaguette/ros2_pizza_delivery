1. Launch the map in 1st terminal with:

export TURTLEBOT3_MODEL="burger"

ros2 launch gazebo_ros gazebo.launch.py world:="~/Downloads/map.sdf"

2. Launch the state publisher in another terminal to get information from nodes

export TURTLEBOT3_MODEL="burger"

ros2 launch turtlebot3_bringup turtlebot3_state_publisher.launch.py use_sim_time:=true

3. Launch aruco node to get position and to scan the qr code.

source install/setup.bash

ros2 run ros2_aruco aruco_node

4. Launch the cartographer

ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

5. Launch the teleop 

export TURTLEBOT3_MODEL="burger"

ros2 run turtlebot3_teleop teleop_keyboard

6. Run our node with

ros2 run ros2_pizza_delivery pizza_pos_node