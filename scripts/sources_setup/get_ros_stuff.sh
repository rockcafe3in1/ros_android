#!/bin/bash

# Abort script on any failures
set -e

if [ $# != 1 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 prefix_path"
    echo "  example: $0 /home/user/my_workspace"
    exit 1
fi

cmd_exists git || die 'git was not found'

ws_prefix=$1

[ "$CMAKE_PREFIX_PATH" = "" ] && die 'could not find target basedir. Have you run build_catkin.sh and sourced setup.bash?'

#cd $CMAKE_PREFIX_PATH
pushd $ws_prefix
mkdir -p $ws_prefix/catkin_ws/src # && cd catkin_ws

if [ -f $ws_prefix/catkin_ws/src/.rosinstall ]; then
  pushd $ws_prefix/catkin_ws/src/
  wstool merge $ROS_ANDROID_ROOT/ndk.rosinstall --merge-replace
  wstool update
  popd
else
  wstool init -j$PARALLEL_JOBS src $ROS_ANDROID_ROOT/ndk.rosinstall
fi

popd # $ws_prefix