#!/bin/bash

PROJECT_DIR=~/ANPL

install-android-core.sh
install-android-opencv.sh

cd ${PROJECT_DIR}/rosjava/src/android_core
echo "include \"android_camera_driver\"">> settings.gradle

#clone android_sensor_driver
git clone https://github.com/talregev/android_camera_driver.git

cd ${PROJECT_DIR}/rosjava
catkin_make
