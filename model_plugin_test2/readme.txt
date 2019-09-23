Gazebo model plugin example 2: apply force via ROS2 topic.

Apply force:
$ ros2 topic pub /control_in_force geometry_msgs/msg/Vector3 "{x: 10.0, y: 0.0, z: 0.0}"

Get current position:
$ ros2 topic echo /current_position

