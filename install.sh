#!/bin/bash
source .checkVersion.sh
rosws="$ROS_PACKAGE_PATH"
rosws="${rosws%%:*}"
echo $rosws
git pull --all
git checkout devel
#cp -r $(ls --ignore install.sh --ignore README.md) $rosws
sudo apt-get install ros-$ros_distro-hokuyo-node \
					 ros-$ros_distro-cv-bridge \
					 ros-$ros_distro-camera-calibration \
					 ros-$ros_distro-image-proc \
