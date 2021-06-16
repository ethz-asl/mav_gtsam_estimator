# mav_state_estimation
This package implements a GTSAM based state estimation framework.

At the moment it supports
- IMU
- GNSS position measurements
- GNSS moving baseline measurements

Please cite our accompanying publication when using it.
```
Bähnemann, Rik, et al.
"Under the Sand: Navigation and Localization of a Small Unmanned Aerial Vehicle for Landmine Detection with Ground Penetrating Synthetic Aperture Radar"
Field Robotics. 2021.
```
![](https://user-images.githubusercontent.com/11293852/104023073-6af65a80-51c1-11eb-8a77-a9cfd53860fd.png)

## Installation on Ubuntu 18.04 and ROS melodic
Install [ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu).

Add [GTSAM](https://gtsam.org/get_started/) PPA repository:
```
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
```

Create a merged catkin workspace.
```
sudo apt install -y python-catkin-tools
cd ~
mkdir -p catkin_ws/src
cd catkin_ws
catkin init
catkin config --extend /opt/ros/melodic
catkin config --merge-devel
```

Install [piksi_multi_cpp](https://github.com/ethz-asl/ethz_piksi_ros/tree/master/piksi_multi_cpp) within the same workspace.

Download this package.
```
cd ~/catkin_ws/src
git clone git@github.com:rikba/mav_state_estimation.git
```

Install all [PPA dependencies](install/prepare-jenkins-slave.sh).
```
./mav_state_estimation/install/prepare-jenkins-slave.sh
```

Next download all individual ROS package dependencies.
**Note**: If you have not setup [SSH keys in GitHub](https://help.github.com/en/enterprise/2.16/user/articles/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) use [dependencies_https.rosinstall](install/dependencies_https.rosinstall).
```
wstool init
wstool merge mav_state_estimation/install/dependencies.rosinstall
wstool update
```

Finally, build the workspace.
```
catkin build
```
