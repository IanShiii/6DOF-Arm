# 6DOF-Arm
## Requirements
- [Ubuntu 24.04](https://releases.ubuntu.com/jammy/)
- [ROS2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/index.html)
- [ROS2 Control (Jazzy Branch)](https://control.ros.org/jazzy/index.html)
- [Moveit2 (Jazzy Branch)](https://moveit.picknik.ai/main/index.html)

## Setup
1. Clone this repo
2. cd to the root of this repo
3. Run [rosdep](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html#rosdep-operation) to install all missing packages.
```
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
``` 
4. At the root of this repo, run `colcon build`.

## Launch
### Instructions
1. Open a new terminal run `source install/setup.bash` at the root of your workspace.
2. Launch using `ros2 launch arm_bringup arm_bringup.launch.py`. 
This launches everything required to control the arm, along with the foxglove_bridge and rosbridge.
