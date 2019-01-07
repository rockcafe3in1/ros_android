#!/bin/bash

# Abort script on any failures
set -e

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

if [ $# != 3 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 system_deps_rosinstall library_prefix_path files_prefix_path"
    echo "  example: $0 /home/user/ros_android/system_deps.rosinstall /home/user/my_workspace/output/libs /home/user/ros_android/files"
    echo $@
    exit 1
fi

echo
echo -e '\e[34mGetting system libraries.\e[39m'
echo

rosinstall_file=$1
lib_prefix=$(cd $2 && pwd)
files_prefix=$(cd $3 && pwd)

# Get everything
if [ -f $lib_prefix/.rosinstall ]; then
  pushd $lib_prefix
  wstool merge $rosinstall_file --merge-replace
  wstool update -j$PARALLEL_JOBS
  popd
else
  wstool init -j$PARALLEL_JOBS $lib_prefix $rosinstall_file
fi

# Library-specific patches / actions.

# Boost
pushd $lib_prefix/boost
[ -d build/out ] || bash -x ./build-android.sh $ANDROID_NDK_HOME --boost=1.68.0 --arch=$ANDROID_ABI
popd

# Rospkg
cp -r $lib_prefix/rospkg $files_prefix
