Taperon – ROS 2 Simulation (Gazebo + ros2_control)

This repository contains the ROS 2 packages required to simulate the Taperon wall-climbing robot in Gazebo (Ignition).
It includes:

taperon_description → URDF, meshes, and ros2_control configuration

taperon_bringup → Launch files, world files, parameters, and controllers

The packages are fully compatible with ROS 2 Humble and Gazebo Fortress/Garden/Ignition.

1. System Requirements
Ubuntu 22.04
ROS 2 Humble installed
Gazebo (Ignition/Fortress)
colcon
xacro

2. Clone the Repository

Create a workspace and clone the repo:

mkdir -p ~/taperon_ws/src

cd ~/taperon_ws/src

git clone https://github.com/ManuelRBX/Taperon.git

3. Build the Workspace

cd ~/taperon_ws

colcon build --symlink-install

Source your environment:

source install/setup.bash

4. Launch Taperon in Gazebo

ros2 launch taperon_bringup taperon_gz.launch.py


This will:
Start Gazebo/Ignition
Spawn the Taperon URDF
Load ros2_control
Load diff_drive controller
Spawn the robot in the provided world
