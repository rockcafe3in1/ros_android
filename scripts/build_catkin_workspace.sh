#!/bin/bash
# Builds catkin workspace with ROS packages and catkinized dependencies.
# See help for required arguments. 
#
# Required environment variables:
# - BASE_DIR: where android.toolchain.cmake is located.
# - SCRIPT_DIR: where utility scripts are located.
# - OUTPUT_DIR: default output directory if path argument is not set.

# print a help screen
function print_help {
    echo "Usage: $0 [options] -p OUTPUT_DIR [catkin build args...]"
    echo "Options:"
    echo "  -p --path <path> target path to build (required)"
    echo "  -b --build-type <cmake_build_type> build binaries with the corresponding cmake build flag"
    echo "                                     Release (default) / Debug / RelWithDebInfo"
    echo "  -v --verbose [<val>] output more verbose error messages"
    echo "                       0: normal (default) / 1: verbose"
    echo "  -h --help display this help screen."
    echo
    echo "Example:"
    echo "    $0 -p /home/user/ros_android/output"
    echo
}

# if no arguments are given print the help screen and exit with error
if [ $# == 0 ]; then
    print_help
    exit 1
fi

# default values
CMAKE_BUILD_TYPE=Release
VERBOSE=""

# source utilities to our environment
source $SCRIPT_DIR/utils.sh


# process options
CATKIN_ARGS=()
while [[ $# > 0 ]]
do
    key="$1"
    case $key in
        -b|--build-type)
            CMAKE_BUILD_TYPE=${2?"Usage: $0 -b <CMAKE_BUILD_TYPE>"}
            shift # past argument
        ;;
        -v|--verbose)
            VERBOSE="--verbose --interleave-output --parallel-packages 1"
            if [ "$2" = "0" ]; then
                VERBOSE=""
                shift # past argument
            elif [ "$2" = "1" ]; then
                shift # past argument
            fi
        ;;
        -h|--help)
            print_help
            exit 0
        ;;
        -p|--path)
            OUTPUT_DIR=${2?"Usage: $0 -p <OUTPUT_DIR>"}
            shift # past argument
        ;;
        *)
            CATKIN_ARGS+=("$1")
        ;;
    esac
    shift # past argument or value
done

# Abort script on any failures
set -e

if [ -z "$OUTPUT_DIR" ]; then
  print_help
  exit 1
fi
: ${RBA_TOOLCHAIN:=$BASE_DIR/android.toolchain.cmake}

# get the prefix path
prefix=$(cd $OUTPUT_DIR && pwd)
TARGET_PATH=$prefix/target

python=$(which python)
python_lib=/usr/lib/x86_64-linux-gnu/libpython2.7.so
python_inc=/usr/include/python2.7
python2_inc=/usr/include/x86_64-linux-gnu/python2.7

cd $prefix/catkin_ws

echo
echo -e '\e[34mRunning catkin build.\e[39m'
echo

catkin config \
  --no-extend \
  --install-space $TARGET_PATH \
  --install \
  --isolate-devel \
  --cmake-args \
    -DCMAKE_TOOLCHAIN_FILE=$RBA_TOOLCHAIN \
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} \
    -DCMAKE_FIND_ROOT_PATH=$prefix \
    -DANDROID_ABI=${ANDROID_ABI} -DANDROID_PLATFORM=${ANDROID_PLATFORM} -DANDROID_STL=${ANDROID_STL} \
    -DPYTHON_EXECUTABLE=$python -DPYTHON_LIBRARY=$python_lib \
    -DPYTHON_INCLUDE_DIR=$python_inc -DPYTHON_INCLUDE_DIR2=$python2_inc \
    -DBUILD_SHARED_LIBS=0 \
    -DBoost_NO_BOOST_CMAKE=ON -DBOOST_ROOT=$TARGET_PATH -DANDROID=TRUE \
    -DBOOST_INCLUDEDIR=$TARGET_PATH/include/boost -DBOOST_LIBRARYDIR=$TARGET_PATH/lib \
    -DBUILD_TESTING=OFF -DCATKIN_ENABLE_TESTING=OFF

catkin build --force-cmake --summary $VERBOSE "${CATKIN_ARGS[@]}"
