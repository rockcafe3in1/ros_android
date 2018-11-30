#!/bin/bash

# Abort script on any failures
set -e

# Define the number of simultaneous jobs to trigger for the different
# tasks that allow it. Use the number of available processors in the
# system.
export PARALLEL_JOBS=$(nproc)

if [[ -f /opt/ros/kinetic/setup.bash ]] ; then
    source /opt/ros/kinetic/setup.bash
else
    echo "ROS environment not found, please install it"
    exit 1
fi

my_loc="$(cd "$(dirname $0)" && pwd)"
export ROS_ANDROID_ROOT=$my_loc
source $my_loc/scripts/util/config.sh
source $my_loc/scripts/util/utils.sh
debugging=0
skip=0
portable=0
help=0
export PATH=$PATH:$my_loc/scripts/build:$my_loc/scripts/sources_setup:$my_loc/scripts/util

# verbose is a bool flag indicating if we want more verbose output in
# the build process. Useful for debugging build system or compiler errors.
verbose=0


if [[ $# -lt 1 ]] ; then
    help=1
fi

for var in "$@"
do
    if [[ ${var} == "--help" ]] ||  [[ ${var} == "-h" ]] ; then
        help=1
    fi
    if [[ ${var} == "--skip" ]] ; then
        skip=1
    fi

    if [[ ${var} == "--debug-symbols" ]] ; then
        debugging=1
    fi

    if [[ ${var} == "--portable" ]] ; then
        portable=1
    fi
done

if [[ $help -eq 1 ]] ; then
    echo "Usage: $0 prefix_path [-h | --help] [--skip] [--debug-symbols]"
    echo "  example: $0 /home/user/my_workspace"
    exit 1
fi

if [[ $skip -eq 1 ]]; then
   echo "-- Skiping projects update"
else
   echo "-- Will update projects"
fi

if [[ $debugging -eq 1 ]]; then
   echo "-- Building workspace WITH debugging symbols"
else
   echo "-- Building workspace without debugging symbols"
fi

if [ ! -d $1 ]; then
    mkdir -p $1
fi

prefix=$(cd $1 && pwd)

if [ -z $ANDROID_NDK ] ; then
    die "ANDROID_NDK ENVIRONMENT NOT FOUND!"
fi

if [ -z $ROS_DISTRO ] ; then
    die "HOST ROS ENVIRONMENT NOT FOUND! Did you source /opt/ros/kinetic/setup.bash"
fi

[ -d $standalone_toolchain_path ] || run_cmd setup_standalone_toolchain


export RBA_TOOLCHAIN=$ANDROID_NDK/build/cmake/android.toolchain.cmake
export CMAKE_PREFIX_PATH=$prefix/target

run_cmd get_system_libraries $prefix

run_cmd get_patched_ros_workspace $skip $my_loc/patches

run_cmd build_pluginlib_support $my_loc/files $prefix

run_cmd build_system_libraries $prefix

echo
echo -e '\e[34mCross-compiling ROS.\e[39m'
echo


if [[ $debugging -eq 1 ]];then
    echo "Build type = DEBUG"
    run_cmd build_cpp -p $prefix -b Debug -v $verbose
else
    echo "Build type = RELEASE"
    run_cmd build_cpp -p $prefix -b Release -v $verbose
fi

echo
echo -e '\e[34mSetting up ndk project.\e[39m'
echo

run_cmd setup_ndk_project $prefix/roscpp_android_ndk $portable

echo
echo -e '\e[34mCreating Android.mk.\e[39m'
echo

# Library path is incorrect for urdf.
# TODO: Need to investigate the source of the issue
sed -i 's/set(libraries "urdf;/set(libraries "/g' $CMAKE_PREFIX_PATH/share/urdf/cmake/urdfConfig.cmake

run_cmd create_android_mk $prefix/catkin_ws/src $prefix/roscpp_android_ndk

if [[ $debugging -eq 1 ]];then
    sed -i "s/#LOCAL_EXPORT_CFLAGS/LOCAL_EXPORT_CFLAGS/g" $prefix/roscpp_android_ndk/Android.mk
fi

# copy Android makfile to use in building the apps with roscpp_android_ndk
cp $my_loc/files/Android.mk.move_base $prefix/roscpp_android_ndk/Android.mk

echo
echo -e '\e[34mCreating sample app.\e[39m'
echo

( cd $prefix && run_cmd sample_app sample_app $prefix/roscpp_android_ndk )

echo
echo -e '\e[34mBuilding apk.\e[39m'
echo

(cd $prefix/sample_app && ant debug)

echo
echo -e '\e[34mCreating move_base sample app.\e[39m'
echo

( cd $prefix && run_cmd sample_app move_base_app $prefix/roscpp_android_ndk )

echo
echo -e '\e[34mBuilding move_base apk.\e[39m'
echo

(cd $prefix/move_base_app && ant debug)

echo
echo -e '\e[34mCreating pluginlib sample app.\e[39m'
echo

( cd $prefix && run_cmd sample_app pluginlib_sample_app $prefix/roscpp_android_ndk )

echo
echo -e '\e[34mBuilding apk.\e[39m'
echo

(cd $prefix/pluginlib_sample_app && ant debug)

echo
echo -e '\e[34mCreating nodelet sample app.\e[39m'
echo

( cd $prefix && run_cmd sample_app nodelet_sample_app $prefix/roscpp_android_ndk )

echo
echo -e '\e[34mBuilding apk.\e[39m'
echo

(cd $prefix/nodelet_sample_app && ant debug)

echo
echo -e '\e[34mCreating image transport sample app.\e[39m'
echo

# Copy specific Android makefile to build the image_transport_sample_app
# This makefile includes the missing opencv 3rd party libraries.
( cp $my_loc/files/Android.mk.image_transport $prefix/roscpp_android_ndk/Android.mk)

( cd $prefix && run_cmd sample_app image_transport_sample_app $prefix/roscpp_android_ndk )

echo
echo -e '\e[34mBuilding apk.\e[39m'
echo

(cd $prefix/image_transport_sample_app && ant debug)


echo
echo 'done.'
echo 'summary of what just happened:'
echo '  target/      was used to build static libraries for ros software'
echo '    include/   contains headers'
echo '    lib/       contains static libraries'
echo '  roscpp_android_ndk/     is a NDK sub-project that can be imported into an NDK app'
echo '  sample_app/  is an example of such an app, a native activity'
echo '  sample_app/bin/sample_app-debug.apk  is the built apk, it implements a subscriber and a publisher'
echo '  move_base_sample_app/  is an example app that implements the move_base node'
echo '  move_base_app/bin/move_base_app-debug.apk  is the built apk for the move_base example'
echo '  pluginlib_sample_app/  is an example of an application using pluginlib'
echo '  pluginlib_sample_app/bin/pluginlib_sample_app-debug.apk  is the built apk'
