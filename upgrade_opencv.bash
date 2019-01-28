#!/bin/bash

opencv_version=4.0.1

wget "https://github.com/opencv/opencv/archive/${opencv_version}.zip" -O opencv.zip
unzip opencv.zip
rm -f opencv.zip
cd opencv-${opencv_version}
mkdir build
cd build
cmake ..
make
make install
