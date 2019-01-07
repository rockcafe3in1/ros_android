cmake_minimum_required(VERSION 2.8.3)
project(hello_ros)

find_package(catkin REQUIRED rosjava_build_tools)

# Set the gradle targets you want catkin's make to run by default, e.g.
# catkin_rosjava_setup(installDist)
catkin_android_setup(assembleDebug)

catkin_package()