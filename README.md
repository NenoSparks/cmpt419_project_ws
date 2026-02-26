# Verify container built correctly
From /workspaces/cmpt419_project_ws/ run: 
```
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```

If that works then run:
```
export MAKE_FLAGS="-j1"
export CMAKE_BUILD_PARALLEL_LEVEL=1
export CXXFLAGS="-O1 -g0"
colcon build --symlink-install --packages-select moveit2_tutorials --parallel-workers 1
source install/setup.bash
ros2 launch moveit2_tutorials demo.launch.py
```
Try moving the robot arm to a goal pose and hit "Plan & Execute". You can also verify the reachability of the robot is correct by trying to move it in directions it isn't physically capable of reaching as the goal pose. You should observe that the robot refuses to move.