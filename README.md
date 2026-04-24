<img width="865" height="596" alt="image" src="https://github.com/user-attachments/assets/c9c9f55a-66a4-4650-bf1d-ad7664262c69" />
# Overview
A ROS2 Humble pick-and-place framework for Universal Robots manipulators using Python and MoveIt2 via pymoveit2. This repository includes a custom script that handles picking up and placing of objects, code to run and launch the robot inside of Gazebo, RViz, and URsim, and setup instructions for connecting and running nodes on physical UR hardware.
# Dev Container
Clone the above repository and open using VS Code dev containers. Copy the packages inside of "Packages" into /src. Then build the workspace and source your workspace. Remember to always source any new terminals you may have to open throughout this tutorial.
```
colcon build --symlink-install
source install/setup.bash
```
# Gazebo Simulation and RViz
In your sourced terminal, run the following command to launch Gazebo and RViz with the MoveIt2 motion planning UI configured.
```
ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py
```
# Launch Custom Python Node
Open a new terminal, source it and then run the following command to launch the custom python script. in 'main()' there are 3 functions, uncomment whichever demo you would like to run.
```
ros2 run ur_pick_and_place_executor pick_place_node
```

# Set up URsim
If you don't current have access to a physical robot, but would like to test your networking with a "simulated" robot, URsim's interface is equivalent to connecting to a real UR robot. For full details please see [here](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/ursim_docker.html).
## Step 1:
To connect to a UR robot using ROS2 we have to use a URcap. Download your desired version from the [UR ExternalControl URCap repo](https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases).
## Step 2:
Create a directory to store the URcap:
```
mkdir -p ${HOME}/.ursim/programs
mkdir -p ${HOME}/.ursim/urcaps
```
## Step 3:
"Install" the URCap by placing the .jar file inside the urcaps folder:
```
URCAP_VERSION=1.0.5 # latest version as if writing this
curl -L -o ${HOME}/.ursim/urcaps/externalcontrol-${URCAP_VERSION}.jar \
  https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/download/v${URCAP_VERSION}/externalcontrol-${URCAP_VERSION}.jar
```
## Step 4:
Inside a terminal on your **host** machine run the following command to start URsim:
```
docker run --rm -it -p 5900:5900 -p 6080:6080 -v ${HOME}/.ursim/urcaps:/urcaps -v ${HOME}/.ursim/programs:/ursim/programs --name ursim universalrobots/ursim_e-series
```
## Step 5:
Open the vnc viewer in your browser, after running URsim using docker, output of what URL to use to access URsim through your browser should be provided. For example http://198.64.0.1:6080/vnc.html?host=198.64.0.1&port=6080. Note down the IP address assigned to URsim by your computer.
## Step 6:
Once on the URsim page, navigate to the settings dashboard by clicking on the the 3 horizontal lines on the top right of the screen. Click on System and add the External Control URCap. If it says that a newer version is already installed, then URsim has already recognized the URCap. Exit System settings and navigate to the Installation tab on the top left.
## Step 7
Click on URCaps -> External Control and enter the IP of your host (if you enabled networking to pass through your container) and leave the custom port as 50002.
## Step 8
Finally, go to Program and add External Control as a Robot Program. The programs name should be "Control by {HOST_IP_ADDRESS}". When you are finally ready to try controlling the robot through URSim press the play button in the bottom right corner and select "Control by {HOST_IP_ADDRESS}" as the program you want to run. Otherwise, when you attempt to run MoveIt2 or custom scripts it will not successfully execute the commands even though the backend successfully connects to the robot!

# Running custom nodes on URSim/Hardware
In the container terminal run:
```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:={URsim_IP}
```
The robot pose inside RViz should match the robot pose in URSim if everything is running correctly. Try manually controlling/jogging the robot in URsim to check whether the mode in RViz is also updating its pose.
In another terminal, source then run:
```
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
```
This will launch RViz along with the MoveIt2 interface. Trying using the MoveIt2 interface on RViz to move the robot to a new position. Make sure it is also moving/updating on URSim. If all this is successful then we have communication from our container to the robot and from the robot back to our container. Congratulations!!! You're finally ready to run your custom nodes on a UR robot!
In a third terminal, source then run one of the pick-and-place demos:
```
ros2 run ur_pick_and_place_executor pick_place_node 
```
# Running on Hardware:
Universal Robots recommends using an Ubuntu system configured with real-time capabilities. [Here](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/real_time.html) is a guide used to setup a PREEMPT_RT kernel, although the simpler option of using a low latency kernal may be worth trying before investing the time to configure the real-time kernel.
