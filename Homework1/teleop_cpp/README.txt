This project demonstrates ROS 2 publisher–subscriber communication by creating a multi-node control system for the Turtlesim simulator.
The system allows both manual keyboard control and autonomous navigation using a Stanley controller following a path generated at runtime.

NODES:
1. virdult_teleop_key
Manual teleoperation node that:
	Publishes velocity commands (geometry_msgs/msg/Twist) to /turtle1/cmd_vel
	Publishes mode status (std_msgs/msg/Bool) to /autonomous_mode
	Switches between manual and autonomous mode using the X key
	KEYS: 
	W -> Move Forward
	S -> Move Backwards
	A -> Turn Left
	D -> Turn Right
	X -> Toggle Manuel/Autonomous

2. path_publisher
Subscribes to /turtle1/pose to track the turtle’s position
Subscribes to /autonomous_mode to detect when autonomous mode is toggled
Publishes a path (nav_msgs/msg/Path) to /path when autonomous mode is ON
Locks the current turtle position as the center and generates a square (or circular) path

3. stanley_controller
Subscribes to:
	/path (from path_publisher)
	/turtle1/pose
	/autonomous_mode
Publishes velocity commands (/turtle1/cmd_vel) when in autonomous mode
Uses the Stanley Control Law to follow the published path

Stanley Controller Explanation:
The Stanley controller is a lateral control algorithm that minimizes two errors:
1.Heading error — difference between the turtle’s heading and the path’s tangent direction.
2. Cross-track error (CTE) — perpendicular distance from the turtle to the path.

Control Law: [\delta = \theta_e + \tan^{-1}\left(\frac{k \times cte}{v}\right)]

Where:
	( \delta ): steering angle (mapped to angular.z)
	( \theta_e ): heading error
	( cte ): cross-track error
	( v ): linear velocity
	( k ): control gain
	
Build Instructions
# Go to your workspace
cd ~/ros2_ws

# Build only this package
colcon build --packages-select teleop_cpp

# Source your workspace (for zsh)
source install/setup.zsh

Run Instructions
Open three terminals:
Terminal 1 — Start Turtlesim:
	ros2 run turtlesim turtlesim_node
Terminal 2 — Start the Teleoperation Node: 
	ros2 run teleop_cpp virdult_teleop_key
Terminal 3 — Start the Path Publisher and Stanley Controller:
	ros2 run teleop_cpp path_publisher
	ros2 run teleop_cpp stanley_controller
	
ROS Topics Summary
1:
	Topic: /turtle1/cmd_vel
	Type: geometry_msgs/msg/Twist
	Publisher: virdult_teleop_key, stanley_controller
	Subscriber: turtlesim_node
2:
	Topic: /turtle1/pose
	Type: turtlesim/msg/Pose
	Publisher: turtlesim_node
	Subscriber: path_publisher, stanley_controller
3:
	Topic: /path
	Type: nav_msgs/msg/Path
	Publisher: path_publisher
	Subscriber: stanley_controller
4:
	Topic: /autonomous_mode
	Type: std_msgs/msg/Bool
	Publisher: virdult_teleop_key
	Subscriber: path_publisher, stanley_controller
	
Troubles that will occur in your computer:
No Keyboard Input: 
	Run with sudo, or change keyboard_device_ in virdult_teleop_key.cpp 
	to the correct /dev/input/event* path
	-> Line 43

Author: Virdult

ROS 2 Homework – Turtlesim Teleoperation & Stanley Controller
Built with ROS 2 Humble on Ubuntu
