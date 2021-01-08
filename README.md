# mav_state_estimation
This package implements a GTSAM based state estimation framework.

At the moment it supports
- IMU
- GNSS position measurements
- GNSS moving baseline measurements

Please cite our accompanying publication when using it.
```
Bähnemann, Rik, et al.
"One Foot Underground: GNSS State Estimation for Ground Penetrating Radar on a Micro Aerial Vehicle."
Field Robotics. 2021.
```

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

Download this package and the Piksi GNSS driver [ethz_piksi_ros](git@github.com:ethz-asl/ethz_piksi_ros.git).
```
cd ~/catkin_ws/src
git clone git@github.com:rikba/mav_state_estimation.git
git clone git@github.com:ethz-asl/polygon_coverage_planning.git
```

Install all [PPA dependencies](install/prepare-jenkins-slave.sh).
```
cd mav_state_estimation
./install/prepare-jenkins-slave.sh
```

Next download all ROS package dependencies.
**Note**: If you have not setup [SSH keys in GitHub](https://help.github.com/en/enterprise/2.16/user/articles/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) use [dependencies_https.rosinstall](install/dependencies_https.rosinstall).
```
wstool init
wstool merge polygon_coverage_planning/install/dependencies.rosinstall
wstool update
```


Download package dependencies from [dependencies.rosinstall](install/dependencies.rosinstall).
```
cd ~/catkin_ws/src
git clone git@github.com:ethz-asl/polygon_coverage_planning.git
wstool init
wstool merge polygon_coverage_planning/install/dependencies.rosinstall
wstool update
```


Install all [remaining dependencies](https://github.com/ethz-asl/polygon_coverage_planning/blob/master/install/prepare-jenkins-slave.sh):
```
cd ~/catkin_ws/polygon_coverage_planning/install
./prepare-jenkins-slave.sh
```

Finally, build the workspace.
```
catkin build
```
