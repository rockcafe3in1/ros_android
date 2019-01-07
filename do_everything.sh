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

# Get the android ndk build helper script
# If file doesn't exist, then download and patch it
#if ! [ -e $prefix/android.toolchain.cmake ]; then
#    cd $prefix
#    download 'https://raw.githubusercontent.com/taka-no-me/android-cmake/556cc14296c226f753a3778d99d8b60778b7df4f/android.toolchain.cmake'
#    patch -p0 -N -d $prefix < $my_loc/patches/android.toolchain.cmake.patch
#    cat $my_loc/files/android.toolchain.cmake.addendum >> $prefix/android.toolchain.cmake
#fi

export RBA_TOOLCHAIN=$ANDROID_NDK_HOME/build/cmake/android.toolchain.cmake
apply_patch $my_loc/patches/android.toolchain.cmake.patch -d $ANDROID_NDK_HOME/build/cmake

# Get all library dependencies.
run_cmd get_system_dependencies $my_loc/system_deps.rosinstall $prefix/libs $my_loc/files

echo
echo -e '\e[34mGetting ROS packages\e[39m'
echo

if [[ $skip -ne 1 ]] ; then
    run_cmd get_catkin_packages $prefix

    echo
    echo -e '\e[34mApplying patches.\e[39m'
    echo

    # patch CMakeLists.txt for lz4 library - Build as a library
    apply_patch $my_loc/patches/lz4.patch

    # patch rosbag_storage - Fix static linking due to missing BZIP2 dependency
    apply_patch $my_loc/patches/rosbag_storage.patch

    # Patch collada - Build as static lib
    apply_patch $my_loc/patches/collada_dom.patch

    #  Patch assimp - Build as static lib
    apply_patch $my_loc/patches/assimp.patch

    # Patch console_bridge - Disable unit tests (unsatisfied dependencies)
    apply_patch $my_loc/patches/console_bridge.patch

    # Patch urdfdom - Build as static lib
    apply_patch $my_loc/patches/urdfdom.patch

    # Patch qhull - Don't install shared libraries
    # TODO: Remove shared libraries to avoid hack in parse_libs.py
    # apply_patch /opt/roscpp_android/patches/qhull.patch

    # Patch bfl - Build as static lib
    apply_patch $my_loc/patches/bfl.patch

    # Patch orocos_kdl - Build as static lib
    apply_patch $my_loc/patches/orocos_kdl.patch

    # Patch PCL - Disable optionals.
    apply_patch $my_loc/patches/pcl-1.8.1.patch

    # Patch uuid - Avoiding stdlib.h include
    apply_patch $my_loc/patches/uuid.patch

    # Patch yaml - Avoid building tests
    apply_patch $my_loc/patches/yaml-cpp.patch

    # Patch bullet - Avoid building examples
    apply_patch $my_loc/patches/bullet.patch

    ## ROS patches

    # Patch rosconsole - Add android backend
    apply_patch $my_loc/patches/rosconsole.patch

    # Patch catkin - Fix transitive linking of interface libraries for static builds
    apply_patch $my_loc/patches/catkin.patch

    # Patch map_server - Fix find yaml
    apply_patch $my_loc/patches/map_server.patch

    # Patch bondcpp - Fix transitive linking problems
    apply_patch $my_loc/patches/bondcpp.patch

    # Patch image_publisher - Fix linking problems, transitive linking,
    # and changed shared to static library building.
    apply_patch $my_loc/patches/image_publisher.patch

    # Patch image_rotate - Fix find opencv and transitive linking problem
    apply_patch $my_loc/patches/image_rotate.patch

    # Patch opencv - Fix installation path
    apply_patch $my_loc/patches/opencv.patch

    # Patch actionlib - problems with Boost changes.
    apply_patch $my_loc/patches/actionlib.patch

    # Patch rospack - problems with Boost changes
    # Also emptied some unnecessary functions to avoid problems related to including Python.
    apply_patch $my_loc/patches/rospack.patch

    # Patch xmlrpcpp - problems with Boost changes.
    apply_patch $my_loc/patches/xmlrpcpp.patch

    # Patch roslib - weird issue with rospack.
    # TODO: Need to look further (only on catkin_make_isolated)
    # apply_patch /opt/roscpp_android/patches/roslib.patch

    # Patch collada_parser - cmake detects mkstemps even though Android does not support it
    # TODO: investigate how to prevent cmake to detect system mkstemps
#    apply_patch $my_loc/patches/collada_parser.patch

    # Patch laser_assembler - Remove testing for Android
    # TODO: It seems like there may be a better way to handle the test issues
    # http://stackoverflow.com/questions/22055741/googletest-for-android-ndk
    # apply_patch $my_loc/patches/laser_assembler.patch

    # Patch laser_filters - Remove testing for Android
    # TODO: It seems like there may be a better way to handle the test issues
    # http://stackoverflow.com/questions/22055741/googletest-for-android-ndk
    # https://source.android.com/reference/com/android/tradefed/testtype/GTest.html
    # apply_patch $my_loc/patches/laser_filters.patch

    # Patch camera_info_manager - remove testing for Android
    # TODO: It seems like there may be a better way to handle the test issues
    # http://stackoverflow.com/questions/22055741/googletest-for-android-ndk
    # https://source.android.com/reference/com/android/tradefed/testtype/GTest.html
    apply_patch $my_loc/patches/camera_info_manager.patch

    # Patch camera_calibration_parsers - deleted python things and solved problem finding Boost.
    apply_patch $my_loc/patches/camera_calibration_parsers.patch

    # Patch cv_bridge - fix transitive linking in cv_bridge-extras.cmake
    apply_patch $my_loc/patches/cv_bridge.patch

    # Patch theora_image_transport - fix transitive linking
    apply_patch $my_loc/patches/theora_image_transport.patch

    # Patch robot_pose_ekf - Add bfl library cmake variables, also, remove tests
    # TODO: The correct way to handle this would be to create .cmake files for bfl and do a findpackage(orocos-bfl)
    # apply_patch $my_loc/patches/robot_pose_ekf.patch

    # Patch robot_state_publisher - Add ARCHIVE DESTINATION
    # TODO: Create PR to add ARCHIVE DESTINATION
    apply_patch $my_loc/patches/robot_state_publisher.patch

    # Patch moveit_core - Add fcl library cmake variables
    # TODO: The correct way to handle this would be to create .cmake files for fcl and do a findpackage(fcl)
    #apply_patch $my_loc/patches/moveit_core.patch

    # Patch moveit_core plugins - Add ARCHIVE DESTINATION
    # TODO: PR merged: https://github.com/ros-planning/moveit_core/pull/251
    # Wait for next release to remove (current 0.6.15)
    #apply_patch $my_loc/patches/moveit_core_plugins.patch

    # Patch camera_calibration_parsers - Fix yaml-cpp dependency
    # TODO: PR created: https://github.com/ros-perception/image_common/pull/36
    # apply_patch $my_loc/patches/camera_calibration_parsers.patch

    # Patch image_view - Solved YAML linking problems, and transitive linking.
    apply_patch $my_loc/patches/image_view.patch

    # Patch depth_image_proc - Solved transitive linking problems
    apply_patch $my_loc/patches/depth_image_proc.patch

    # Patch urdf - Fixed linking with pluginlib and dependencies in downstream packages
    apply_patch $my_loc/patches/urdf.patch

    # Patch global_planner - Add angles dependency
    # TODO: PR merged: https://github.com/ros-planning/navigation/pull/359
    # Wait for next release to remove (current 1.12.4)
    # apply_patch $my_loc/patches/global_planner.patch

    #Patch Poco lib
    apply_patch $my_loc/patches/poco.patch

    # Plugin specific patches
    if [ $use_pluginlib -ne 0 ]; then
        # Patch pluginlib for static loading
        apply_patch $my_loc/patches/pluginlib.patch
        # apply_patch image_transport # to fix faulty export plugins
        apply_patch $my_loc/patches/image_transport.patch
    fi

    ## Demo Application specific patches

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
