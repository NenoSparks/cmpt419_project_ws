# Verify container built correctly
From /workspaces/cmpt419_project_ws/ run: 
```
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```

Building moveit2_tutorials from source can be memory intensive. To avoid crashing during
builds, we will increase the allocated memory limit for wsl. 

In the C:\Users\<Username> directory create a .wslconfig file with the following contents:
[wsl2]
memory=10GB
processors=4
swap=16GB

Next, run wsl --shutdown from the powershell and wait 10 seconds before attempting to restart
WSL. Then restart docker and try to open the repository in the container.

If that works then run:
```
export MAKE_FLAGS="-j1"
export CMAKE_BUILD_PARALLEL_LEVEL=1
export CXXFLAGS="-O1 -g0"
colcon build --symlink-install --packages-select moveit2_tutorials --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYTPE=Release
source install/setup.bash
ros2 launch moveit2_tutorials demo.launch.py
```
Try moving the robot arm to a goal pose and hit "Plan & Execute". You can also verify the reachability of the robot is correct by trying to move it in directions it isn't physically capable of reaching as the goal pose. You should observe that the robot refuses to move.