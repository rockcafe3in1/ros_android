#!/bin/bash

# print a help screen
function print_help {
    echo "Usage: $0 [options] -p prefix_path [catkin build args...]"
    echo "Options:"
    echo "  -p --path <path> target path to build (required)"
    echo "  -b --build-type <cmake_build_type> build binaries with the corresponding cmake build flag"
    echo "                                     Release (default) / Debug / RelWithDebInfo"
    echo "  -v --verbose [<val>] output more verbose error messages"
    echo "                       0: normal (default) / 1: verbose"
    echo "  -h --help display this help screen."
    echo
    echo "Example:"
    echo "    $0 -p /home/user/my_workspace"
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

# get the base folder
my_loc="$(cd "$(dirname $0)" && pwd)"

# source utilities to our environment
source $my_loc/config.sh
source $my_loc/utils.sh


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
            TARGET_PATH=${2?"Usage: $0 -p <TARGET_PATH>"}
            shift # past argument
        ;;
        *)
            CATKIN_ARGS+=($1)
        ;;
    esac
    shift # past argument or value
done

# Abort script on any failures
set -e

if [ "$TARGET_PATH" = "" ]; then
  print_help
  exit 1
fi
: ${RBA_TOOLCHAIN:=$ANDROID_NDK/build/cmake/android.toolchain.cmake}
: ${CMAKE_PREFIX_PATH:=$TARGET_PATH}

# get the prefix path
prefix=$(cd $TARGET_PATH && pwd)

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
  --install-space $prefix/target \
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
    -DBoost_NO_BOOST_CMAKE=ON -DBOOST_ROOT=$CMAKE_PREFIX_PATH -DANDROID=TRUE \
    -DBOOST_INCLUDEDIR=$CMAKE_PREFIX_PATH/include/boost -DBOOST_LIBRARYDIR=$CMAKE_PREFIX_PATH/lib \
    -DROSCONSOLE_BACKEND=print \
    -DBUILD_TESTING=OFF -DCATKIN_ENABLE_TESTING=OFF

catkin build --force-cmake --summary $VERBOSE "${CATKIN_ARGS[@]}"
    #-DCMAKE_FIND_ROOT_PATH_MODE_PROGRAM=BOTH -DCMAKE_FIND_ROOT_PATH_MODE_LIBRARY=BOTH -DCMAKE_FIND_ROOT_PATH_MODE_PACKAGE=BOTH \
    #-DBoost_NO_BOOST_CMAKE=TRUE -DBoost_NO_SYSTEM_PATHS=TRUE -DBOOST_ROOT:PATHNAME=$CMAKE_PREFIX_PATH/include/boost \
    #-DBOOST_INCLUDEDIR:PATH=$CMAKE_PREFIX_PATH/include/boost -DBOOST_LIBRARYDIR:PATH=$CMAKE_PREFIX_PATH/lib \
    #-DBoost_USE_STATIC_LIBS=ON -DBoost_NO_BOOST_CMAKE=ON
   # -DBOOST_INCLUDEDIR:PATH=$CMAKE_PREFIX_PATH/include -DBOOST_LIBRARYDIR:PATH=$CMAKE_PREFIX_PATH/lib
   # -DBOOST_ROOT:PATHNAME=$CMAKE_PREFIX_PATH/include
