Gazebo model plugin example 2: apply force via ROS2 topic.

Build:
$ colcon build

Start simulation environments:
$ export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:`pwd`/install/model_plugin_test2/lib/model_plugin_test2
$ gzserver -u worlds/model_plugin_test2.world --verbose

Start simulation:
$ gzclient
 -> Click the start button on the GUI window.

Apply force:
$ ros2 topic pub /control_in_force geometry_msgs/msg/Vector3 "{x: 10.0, y: 0.0, z: 0.0}"

Get current position:
$ ros2 topic echo /current_position

