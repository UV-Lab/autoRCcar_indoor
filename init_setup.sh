#!/bin/bash
### Dependencies

## LIO-SAM (ros2)
sudo apt install -y ros-humble-perception-pcl \
  	   ros-humble-pcl-msgs \
  	   ros-humble-vision-opencv \
  	   ros-humble-xacro

## LIO-SAM (gtsam)
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt install -y libgtsam-dev libgtsam-unstable-dev

## oCam
sudo apt-get install libv4l-dev libudev-dev

## GCS
pip install numpy PyQt5 pyqtgraph

## Livox-SDK2
cd Livox-SDK2
if [ -d "build" ]; then
    rm -rf build
fi
mkdir build && cd build
cmake .. && make -j
sudo make install
cd ../..

./ros2_build.sh
