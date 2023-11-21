#!/bin/bash

# source ros
source /opt/ros/foxy/setup.bash

# update first
apt update

# install dependencies
rosdep install --rosdistro foxy --from-paths src -i -y

# install custom packages
apt install ros-foxy-tf-transformations

# source local
[ -d "install" ] && source install/local_setup.bash

# multiple windows
tmux
