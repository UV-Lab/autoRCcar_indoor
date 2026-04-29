#!/bin/bash

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
source /home/robot-nav/lidar_slam/gac_lidar_slam_loc_ws/install/setup.bash
cmake -DCMAKE_C_COMPILER=clang-14 -DCMAKE_CXX_COMPILER=clang++-14 \
      -DLIBOMP_PATH=/usr/lib/llvm-14/lib \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=1 ..