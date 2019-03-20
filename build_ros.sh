#!/usr/bin/env bash
echo "Building ROS nodes"

cd Examples/ROS/ORB_CARV_Pub
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Debug
make clean
make -j 2
