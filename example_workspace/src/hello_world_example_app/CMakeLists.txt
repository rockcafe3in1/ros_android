cmake_minimum_required(VERSION 2.8.3)
project(hello_ros)

find_package(catkin REQUIRED rosjava_build_tools)

set(ENV{ANDROID_STL} ${ANDROID_STL})
catkin_android_setup(assemble${CMAKE_BUILD_TYPE})

catkin_package()