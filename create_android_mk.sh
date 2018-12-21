#!/bin/bash

# Abort script on any failures
set -e

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

if [ $# != 2 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 project_path"
    echo "  example: $0 /home/user/my_catkin_ws/src /home/user/my_output_library_dir"
    exit 1
fi

[ "$TARGET_DIR" = "" ] && die 'could not find target basedir. Please set $TARGET_DIR environment variable.'
prefix=$(cd $1 && pwd)

# Get list of packages from catkin
package_list=$(/opt/ros/kinetic/bin/catkin_topological_order --only-names $prefix | tr '\n' ';')

# Call a CMAKE script to get the equivalent of $catkin_LIBRARIES for all of the above packages
rm -rf $TARGET_DIR/find_libs
mkdir -p $TARGET_DIR/find_libs
cp $my_loc/files/FindLibrariesCMakeLists.txt $TARGET_DIR/find_libs/CMakeLists.txt
cd $TARGET_DIR/find_libs
cmake ../find_libs -DTARGET_DIR="$TARGET_DIR;$ANDROID_NDK/platforms/android-14/arch-arm/usr/lib" \
             -DALL_PACKAGES="$package_list"

# Read the output file to get the paths of all of the libraries
full_library_list=$(cat $TARGET_DIR/find_libs/libraries.txt)

# Parse this libraries (separated by ;), skip all libraries that start with the second argument paths (separated by ;)
lib_output=$($my_loc/parse_libs.py $full_library_list $ANDROID_NDK/platforms/android-14/arch-arm/usr/lib)

# Go to the output library directory
if [ ! -d $2 ]; then
    mkdir -p $2
fi
cd $2

# Create and Android.mk with the parsed libraries
cp $my_loc/files/tfa/Android.mk.in1 ./Android.mk
echo "$lib_output" >> ./Android.mk
pluginlib_helper_cpp=$my_loc/files/pluginlib_helper/pluginlib_helper.cpp
if [ $use_pluginlib -ne 0 ]; then
  # Replace commented line. Add the pluginlib helper module to the build.
  sed "/^#LOCAL_SRC_FILES/ c LOCAL_SRC_FILES=$pluginlib_helper_cpp" $my_loc/files/tfa/Android.mk.in2 >> ./Android.mk
else
  cat $my_loc/files/tfa/Android.mk.in2 >> ./Android.mk
fi

