#!/bin/bash
# This script uses the following environment variables:
# - OUTPUT_PREFIX: where the ROS workspace shall be created.
# - PARALLEL_JOBS: how many threads to use to build the library.
# - ROS_ANDROID_ROOT: the root directory of the repository.
# - UTIL_DIR: Directory where basic utilities are located.

# Abort script on any failures
set -e

if [ $# != 1 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 prefix_path"
    echo "  example: $0 /home/user/my_workspace"
    exit 1
fi

source $UTIL_DIR/basic_utils.sh
cmd_exists git || die 'git was not found'

[ "$CMAKE_PREFIX_PATH" = "" ] && die 'could not find target basedir. Have you run build_catkin.sh and sourced setup.bash?'

pushd $OUTPUT_PREFIX
mkdir -p $OUTPUT_PREFIX/catkin_ws/src # && cd catkin_ws

if [ -f $OUTPUT_PREFIX/catkin_ws/src/.rosinstall ]; then
  pushd $OUTPUT_PREFIX/catkin_ws/src/
  wstool merge $ROS_ANDROID_ROOT/ndk.rosinstall --merge-replace
  wstool update
  popd
else
  wstool init -j$PARALLEL_JOBS $OUTPUT_PREFIX/catkin_ws/src $ROS_ANDROID_ROOT/ndk.rosinstall
fi

popd # $OUTPUT_PREFIX
