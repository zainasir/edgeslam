#!/bin/sh
# Install zed ros wrapper and update paths. Adding this to the dockerfile does not work due to path issues"

# Install zed ros wrapper
cd /home/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source ./devel/setup.bash

# Build edgeslam
echo "export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:/home/edgeslam/Examples/ROS" >> ~/.bashrc
cd /home/edgeslam
./build_ros.sh
