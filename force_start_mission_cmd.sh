echo -e 'Force start of the mission!'
echo -e 'ros2 topic pub /results std_msgs/msg/Int16MultiArray "{data: [-1, 10, 100]}"'
ros2 topic pub -1 /results std_msgs/msg/Int16MultiArray "{data: [-1, 10, 100]}"
