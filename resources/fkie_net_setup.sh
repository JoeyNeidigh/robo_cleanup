#!/bin/bash

WORKSPACE_SRC=$HOME"/catkin_ws/src/"

if [ ! -d $WORKSPACE_SRC ]; then
    printf "\nINITIALIZING WORKSPACE\n"
    mkdir -p $WORKSPACE_SRC
    cd $WORKSPACE_SRC
    catkin_init_workspace
fi

cp ~/Downloads/multimaster_fkie-master/default_cfg_fkie -r ~/catkin_ws/src/default_cfg_fkie
cp ~/Downloads/multimaster_fkie-master/master_discovery_fkie -r ~/catkin_ws/src/master_discovery_fkie
cp ~/Downloads/multimaster_fkie-master/master_sync_fkie -r ~/catkin_ws/src/master_sync_fkie
cp ~/Downloads/multimaster_fkie-master/multimaster_fkie -r ~/catkin_ws/src/multimaster_fkie
cp ~/Downloads/multimaster_fkie-master/multimaster_msgs_fkie -r ~/catkin_ws/src/multimaster_msgs_fkie
cp ~/Downloads/multimaster_fkie-master/node_manager_fkie -r ~/catkin_ws/src/node_manager_fkie
cp ~/Downloads/multimaster_fkie-master/README.rst ~/catkin_ws/src/
cp ~/Downloads/multimaster_fkie-master/rosdep.yaml ~/catkin_ws/src/

cd ~/catkin_ws
catkin_make
source devel/setup.bash
rospack profile





