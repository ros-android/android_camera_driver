#!/bin/bash

PROJECT_DIR=~/ANPL

#from http://blog.hig.no/gtl/2014/08/28/opencv-and-android-studio/
#downlaod opencv android sdk
cd ~/Downloads
wget -O OpenCV-android-sdk.zip 'http://garr.dl.sourceforge.net/project/opencvlibrary/opencv-android/2.4.10/OpenCV-2.4.10-android-sdk.zip'
#from http://unix.stackexchange.com/questions/59276/how-to-extract-only-a-specific-folder-from-a-zipped-archive-to-a-given-directory
unzip OpenCV-android-sdk.zip "OpenCV-2.4.10-android-sdk/sdk/java/*" -d ~/Downloads
mv OpenCV-2.4.10-android-sdk/sdk/java . 
rm -rf OpenCV-android-sdk.zip OpenCV-2.4.10-android-sdk
mv java opencv
rm -rf ${PROJECT_DIR}/rosjava/src/android_core/opencv
mv opencv ${PROJECT_DIR}/rosjava/src/android_core/
cd ${PROJECT_DIR}/rosjava/src/android_core/opencv

#create build.gradle
cat << EOF > build.gradle

apply plugin: 'android-library'

buildscript {
  repositories {
    mavenCentral()
  }
  dependencies {
    classpath 'com.android.tools.build:gradle:0.12.2'
  }
}

android {
  compileSdkVersion 15
  buildToolsVersion "19.1.0"

  defaultConfig {
    minSdkVersion 8
    targetSdkVersion 15
    versionCode 24100
    versionName "2.4.10"
  }

  sourceSets {
    main {
      manifest.srcFile 'AndroidManifest.xml'
      java.srcDirs = ['src']
      resources.srcDirs = ['src']
      res.srcDirs = ['res']
      aidl.srcDirs = ['src']
    }
  }
}

EOF

cd ..
echo "include \"opencv\"">> settings.gradle

cd ${PROJECT_DIR}/rosjava/
catkin_make
