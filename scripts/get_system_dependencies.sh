#!/bin/bash
# Gets system dependencies in a target directory.
# See help for required positional arguments.

# Required environment variables:
# - SCRIPT_DIR: where utility scripts are located.

# Abort script on any failures
set -e

source $SCRIPT_DIR/utils.sh

if [ $# != 3 ] || [ "$1" == '-h' ] || [ "$1" == '--help' ]; then
    echo "Usage: $0 system_deps_rosinstall library_prefix_path files_prefix_path"
    echo "  example: $0 /home/user/ros_android/system_deps.rosinstall /home/user/my_workspace/output/libs /home/user/ros_android/files"
    echo $@
    exit 1
fi

echo
echo -e '\e[34mGetting system libraries.\e[39m'
echo

cmd_exists wstool || die 'wstool was not found'

rosinstall_file="$1"
lib_prefix=$(cd "$2" && pwd)
files_prefix=$(cd "$3" && pwd)


pushd $lib_prefix
if [ ! -f .rosinstall ]; then
  ln -s $rosinstall_file .rosinstall
fi
wstool update -j$PARALLEL_JOBS
popd

# Library-specific patches / actions.

# Boost
pushd $lib_prefix/boost
[ -d build/out ] || bash -x ./build-android.sh $ANDROID_NDK_HOME --boost=1.68.0 --arch=$ANDROID_ABI
popd

# Rospkg
cp -rn $lib_prefix/rospkg $files_prefix
