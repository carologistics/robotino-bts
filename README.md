# Setup
```
mkdir -p ros2_ws/src
cd ros2_ws
git clone https://github.com/carologistics/robotino-bts.git src/robotino-bts
vcs import --input src/robotino-bts/dependencies.repos src --recursive
colcon build --symlink-install
source install/setup.bash
```

# Launch

```
ros2 launch robotino_behavior_tree server.launch.py
```

# Execute
```
ros2 action send_goal /robotino_behavior_server btcpp_ros2_interfaces/action/ExecuteTree \
"{target_tree: RobotinoDemo}"
```
