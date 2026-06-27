# Setup
```
mkdir -p ros2_ws/src
cd ros2_ws
git clone https://github.com/carologistics/robotino-bts.git src/robotino-bts
vcs import --input src/robotino-bts/dependencies.repos src --recursive
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

# Launch

```
ros2 launch robotino_behavior_tree server.launch.py namespace:=robotinobase1
```

# Execute
```
ros2 action send_goal /robotinobase1/robotino_behavior_server btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: RobotinoDemo, payload: ''}"
```

## ManipulateObject Examples

Replace `machine_input_tf` with the TF frame of the target robot pose for the machine input. The Robotino-specific inputs are passed through the generic `payload` string as a small JSON object, so `BehaviorTree.ROS2` can stay upstream-compatible.

Pick a grey eurobox:
```
ros2 action send_goal /robotinobase1/robotino_behavior_server btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: ManipulateObject, payload: '{\"machine_input_tf\":\"machine_input_tf\",\"reference_frame\":\"base_link\",\"object_type\":\"eurobox\",\"action\":\"pick\"}'}"
```

Put a grey eurobox:
```
ros2 action send_goal /robotinobase1/robotino_behavior_server btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: ManipulateObject, payload: '{\"machine_input_tf\":\"machine_input_tf\",\"reference_frame\":\"base_link\",\"object_type\":\"eurobox\",\"action\":\"put\"}'}"
```

Pick a yellow lego brick:
```
ros2 action send_goal /robotinobase1/robotino_behavior_server btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: ManipulateObject, payload: '{\"machine_input_tf\":\"machine_input_tf\",\"reference_frame\":\"base_link\",\"object_type\":\"lego\",\"lego_color\":\"yellow\",\"action\":\"pick\"}'}"
```

Put a yellow lego brick:
```
ros2 action send_goal /robotinobase1/robotino_behavior_server btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: ManipulateObject, payload: '{\"machine_input_tf\":\"machine_input_tf\",\"reference_frame\":\"base_link\",\"object_type\":\"lego\",\"lego_color\":\"yellow\",\"action\":\"put\"}'}"
```

