# f1tenth-gym-setup
Docker Container setup for F1tenth gym simulator

Lab Slides: [Google Drive](https://drive.google.com/drive/folders/1zkDPOFqiTPDinysS83SIexqvinx8rnK8)

# Set Up

Go into the gym directory you installed the program in.

Run shell script:
```
sh scripts/run.sh
```
Open a web browser and go to link
```
http://localhost:8080/vnc.html
```
Run the ROS within the VM hosted in the docker simulator and run
```
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
```
Launch the simulation
```
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

# Safety Node Setup

Source two environments under /sim_ws/:
```
source /opt/ros/foxy/setup.bash
source install/setup.bash
```
Run the command under /sim_ws/
```
colcon build --packages-select safety_node
```
Source the workspace environment again
```
source install/setup.bash
```
Run the safety_node node
```
ros2 run safety_node safety_node
```

# Wall Follow Setup

Source two environments under /sim_ws/:
```
source /opt/ros/foxy/setup.bash
source install/setup.bash
```
Run the command under /sim_ws/
```
colcon build --packages-select wall_follow
```
Source the workspace environment again
```
source install/setup.bash
```
Run the wall_follow node
```
ros2 run wall_follow wall_follow_node
```

Edited by: Hyunsu Lim
