#!/bin/bash -e
echo "Running the prepare script for mav_state_estimation.";
source /opt/ros/melodic/setup.bash
ROS_VERSION=`rosversion -d`
echo "ROS version: ${ROS_VERSION}"

# Build dependencies.
sudo apt install -y python-wstool python-catkin-tools

# Eigen3
sudo apt install -y libeigen3-dev

# GTSAM.
sudo apt install -y libgtsam-dev libgtsam-unstable-dev

# Package dependencies.
sudo apt install -y ros-${ROS_VERSION}-eigen-conversions
sudo apt install -y ros-${ROS_VERSION}-geometry-msgs
sudo apt install -y ros-${ROS_VERSION}-nav-msgs
sudo apt install -y ros-${ROS_VERSION}-nodelet
sudo apt install -y ros-${ROS_VERSION}-rosbag
sudo apt install -y ros-${ROS_VERSION}-roscpp
sudo apt install -y ros-${ROS_VERSION}-sensor-msgs
sudo apt install -y ros-${ROS_VERSION}-std-msgs
sudo apt install -y ros-${ROS_VERSION}-tf2-geometry-msgs
sudo apt install -y ros-${ROS_VERSION}-tf2-msgs
sudo apt install -y ros-${ROS_VERSION}-tf2-ros
sudo apt install -y ros-${ROS_VERSION}-message-generation
sudo apt install -y ros-${ROS_VERSION}-message-runtime
