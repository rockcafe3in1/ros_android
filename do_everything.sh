#!/bin/bash

# Abort script on any failures
set -e

# Define the number of simultaneous jobs to trigger for the different
# tasks that allow it. Use the number of available processors in the
# system.
export PARALLEL_JOBS=$(nproc)

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh
debugging=0
skip=0
portable=0
help=0
samples=0
user_workspace=""

# verbose is a bool flag indicating if we want more verbose output in
# the build process. Useful for debugging build system or compiler errors.
verbose=0


if [[ $# -lt 1 ]] ; then
    help=1
fi

while [[ $# -gt 0 ]]
do
    if [[ $1 == "--help" ]] ||  [[ $1 == "-h" ]] ; then
        help=1
    elif [[ $1 == "--skip" ]] ; then
        skip=1
    elif [[ $1 == "--debug-symbols" ]] ; then
        debugging=1
    elif [[ $1 == "--portable" ]] ; then
        portable=1
    elif [[ $1 == "--extends-workspace" ]] ; then
        if [ -d $2 ]; then
            user_workspace=$2
        else
            echo "--extends-workspace should be folowed by a directory"
            help=1
        fi
        shift
    elif [[ ${var} == "--samples" ]] ; then
        samples=1
    elif [[ ! -z prefix ]]; then
        if [ ! -d $1 ]; then
            mkdir -p $1
        fi
        prefix=$(cd $1 && pwd)
    fi

    shift
done

if [[ -z prefix ]]; then
    help=1
fi

if [[ $help -eq 1 ]] ; then
    echo "Usage: $0 prefix_path [-h | --help] [--skip] [--debug-symbols] [--extends-workspace path]"
    echo "  example: $0 /home/user/my_workspace"
    echo " --extends-workspace path: Pluginlib will also search in this path for plugins."
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

run_cmd() {
    cmd=$1.sh
    shift
    $my_loc/$cmd $@ || die "$cmd $@ died with error code $?"
}

if [ -z $ANDROID_NDK_HOME ] ; then
    die "ANDROID_NDK_HOME ENVIRONMENT NOT FOUND!"
fi

[ -d $standalone_toolchain_path ] || run_cmd setup_standalone_toolchain

echo
echo -e '\e[34mGetting library dependencies.\e[39m'
echo

mkdir -p $prefix/libs

export TARGET_DIR=$prefix/target
[ -d $TARGET_DIR ] || mkdir -p $TARGET_DIR

export RBA_TOOLCHAIN=$ANDROID_NDK_HOME/build/cmake/android.toolchain.cmake
apply_patch $my_loc/patches/android.toolchain.cmake.patch -d $ANDROID_NDK_HOME/build/cmake

# Get all library dependencies.
run_cmd get_system_dependencies $my_loc/system_deps.rosinstall $prefix/libs $my_loc/files

echo
echo -e '\e[34mGetting ROS packages\e[39m'
echo

if [[ $skip -ne 1 ]] ; then
    run_cmd get_catkin_packages $prefix

    run_cmd apply_patches $my_loc/patches/source_patches $prefix
fi

# Before build
# Search packages that depend on pluginlib and generate plugin loader.
if [ $use_pluginlib -ne 0 ]; then
    echo
    echo -e '\e[34mBuilding pluginlib support...\e[39m'
    echo

    pluginlib_helper_file=pluginlib_helper.cpp
    $my_loc/files/pluginlib_helper/pluginlib_helper.py -scanroot $prefix/catkin_ws/src $user_workspace -cppout $my_loc/files/pluginlib_helper/$pluginlib_helper_file
    cp $my_loc/files/pluginlib_helper/$pluginlib_helper_file $prefix/catkin_ws/src/pluginlib/src/
    line="add_library(pluginlib STATIC src/pluginlib_helper.cpp)"
    # temporally turn off error detection
    set +e
    grep "$line" $prefix/catkin_ws/src/pluginlib/CMakeLists.txt
    # if line is not already added, then add it to the pluginlib cmake
    if [ $? -ne 0 ]; then
        # backup the file
        cp $prefix/catkin_ws/src/pluginlib/CMakeLists.txt $prefix/catkin_ws/src/pluginlib/CMakeLists.txt.bak
        sed -i '/INCLUDE_DIRS include/a LIBRARIES ${PROJECT_NAME}' $prefix/catkin_ws/src/pluginlib/CMakeLists.txt
        echo -e "\n"$line >> $prefix/catkin_ws/src/pluginlib/CMakeLists.txt
        echo 'install(TARGETS pluginlib RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})' >> $prefix/catkin_ws/src/pluginlib/CMakeLists.txt
    fi
    # turn error detection back on
    set -e
fi

echo
echo -e '\e[34mBuilding library dependencies.\e[39m'
echo

# if the library doesn't exist, then build it
[ -f $TARGET_DIR/lib/libbz2.a ] || run_cmd build_library bzip2 $prefix/libs/bzip2
[ -f $TARGET_DIR/lib/libuuid.a ] || run_cmd build_library uuid $prefix/libs/uuid
[ -f $TARGET_DIR/lib/libboost_system.a ] || run_cmd copy_boost $prefix/libs/boost
[ -f $TARGET_DIR/lib/libPocoFoundation.a ] || run_cmd build_library_with_toolchain poco $prefix/libs/poco
[ -f $TARGET_DIR/lib/libtinyxml.a ] || run_cmd build_library tinyxml $prefix/libs/tinyxml
[ -f $TARGET_DIR/lib/libtinyxml2.a ] || run_cmd build_library tinyxml2 $prefix/libs/tinyxml2
[ -f $TARGET_DIR/lib/libconsole_bridge.a ] || run_cmd build_library console_bridge $prefix/libs/console_bridge
[ -f $TARGET_DIR/lib/liblz4.a ] || run_cmd build_library lz4 $prefix/libs/lz4/cmake_unofficial
[ -f $TARGET_DIR/lib/libcurl.a ] || run_cmd build_library_with_toolchain curl $prefix/libs/curl
[ -f $TARGET_DIR/include/urdf_model/model.h ] || run_cmd build_library urdfdom_headers $prefix/libs/urdfdom_headers
[ -f $TARGET_DIR/lib/liburdfdom_model.a ] || run_cmd build_library urdfdom $prefix/libs/urdfdom
[ -f $TARGET_DIR/lib/libiconv.a ] || run_cmd build_library_with_toolchain libiconv $prefix/libs/libiconv
[ -f $TARGET_DIR/lib/libxml2.a ] || run_cmd build_library_with_toolchain libxml2 $prefix/libs/libxml2
[ -f $TARGET_DIR/lib/libcollada-dom2.4-dp.a ] || run_cmd build_library collada_dom $prefix/libs/collada_dom
[ -f $TARGET_DIR/lib/libassimp.a ] || run_cmd build_library assimp $prefix/libs/assimp
[ -f $TARGET_DIR/include/eigen3/signature_of_eigen3_matrix_library ] || run_cmd build_library eigen $prefix/libs/eigen
[ -f $TARGET_DIR/lib/libqhullstatic.a ] || run_cmd build_library qhull $prefix/libs/qhull
[ -f $TARGET_DIR/lib/libyaml-cpp.a ] || run_cmd build_library yaml-cpp $prefix/libs/yaml-cpp
[ -f $TARGET_DIR/lib/libflann_cpp_s.a ] || run_cmd build_library flann $prefix/libs/flann
[ -f $TARGET_DIR/lib/libpcl_common.a ] || run_cmd build_library pcl $prefix/libs/pcl
[ -f $TARGET_DIR/lib/libBulletSoftBody.a ] || run_cmd build_library bullet $prefix/libs/bullet
[ -f $TARGET_DIR/lib/libSDL.a ] || run_cmd build_library_with_toolchain sdl $prefix/libs/sdl
[ -f $TARGET_DIR/lib/libSDL_image.a ] || run_cmd build_library_with_toolchain sdl-image $prefix/libs/sdl-image
[ -f $TARGET_DIR/lib/libogg.a ] || run_cmd build_library_with_toolchain ogg $prefix/libs/ogg
[ -f $TARGET_DIR/lib/libvorbis.a ] || run_cmd build_library_with_toolchain vorbis $prefix/libs/vorbis
[ -f $TARGET_DIR/lib/libtheora.a ] || run_cmd build_library_with_toolchain theora $prefix/libs/theora

echo
echo -e '\e[34mCross-compiling ROS.\e[39m'
echo


if [[ $debugging -eq 1 ]];then
    echo "Build type = DEBUG"
    run_cmd build_catkin_workspace -p $prefix -b Debug -v $verbose
else
    echo "Build type = RELEASE"
    run_cmd build_catkin_workspace -p $prefix -b Release -v $verbose
fi

if [[ $samples -eq 1 ]];then
    echo
    echo -e '\e[34mBuilding sample apps.\e[39m'
    echo

    build_sample() {
        cd $2

        echo "Building $1"

        # TODO(ivanpauno): Add release apk option
        ./gradlew assembleDebug

        mkdir -p $prefix/target/apks/$1

        echo "Copying generated apks inside $prefix/target/apks/$1"
        find . -type f -name "*.apk" -exec cp {} $prefix/target/apks/$1 \;
        cd $my_loc
    }

    [ -d $prefix/target/apks/hello_world ] || build_sample hello_world $my_loc/files/hello_world_example_app
fi
