#!/bin/bash
echo "Configuring and building Thirdparty/eigen-3.1.3 ..."

cd Thirdparty/eigen-3.1.3
rm -rf build
mkdir build
cd build
cmake ..
sudo make install

echo "Configuring and building Thirdparty/Pangolin ..."
sudo apt-get install libglew-dev
sudo apt-get install libglu1-mesa-dev freeglut3-dev mesa-common-dev
sudo apt-get install ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev
sudo apt-get install libdc1394-22-dev libraw1394-dev
sudo apt-get install libjpeg-dev libpng-dev libtiff5-dev libopenexr-dev
sudo apt-get install libboost-all-dev libopenblas-dev
sudo apt-get install libbluetooth-dev

cd ../../Pangolin
rm -rf build
mkdir build
cd build
cmake -DCPP11_NO_BOOST=1 -DEigen3_INCLUDE_DIR=/usr/local/include/eigen3 ..
sudo make -j$(nproc) install

cd ~
git clone https://github.com/lava/matplotlib-cpp.git
cd matplotlib-cpp
mkdir build
cd build
cmake ..
sudo make -j$(nproc) install
sudo pip install matplotlib
sudo pip install numpy

