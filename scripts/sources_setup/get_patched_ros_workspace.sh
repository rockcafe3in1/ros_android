# This script uses the following environment variables:
# - ANDROID_NDK: Android NDK's directory.
# - OUTPUT_PREFIX: where the ROS workspace shall be created.
# - UTIL_DIR: Directory where basic utilities are located.
# Parameters:
# - skip: True if this task has to be skipped
# - patches_dir: Directory where the patch files are.


skip=$1
patches_dir=$(cd $2 && pwd)

if [[ $skip -ne 1 ]] ; then
    source $UTIL_DIR/basic_utils.sh
    source $UTIL_DIR/apply_patch.sh

    echo_title 'Getting ROS packages'
    
    run_cmd get_ros_stuff $OUTPUT_PREFIX

    echo_title 'Applying patches'

    # patch CMakeLists.txt for lz4 library - Build as a library
    apply_patch $patches_dir/lz4.patch

    # patch rosbag_storage - Fix static linking due to missing BZIP2 dependency
    apply_patch $patches_dir/rosbag_storage.patch

    # Patch collada - Build as static lib
    apply_patch $patches_dir/collada_dom.patch

    #  Patch assimp - Build as static lib
    apply_patch $patches_dir/assimp.patch

    # Patch console_bridge - Disable unit tests (unsatisfied dependencies)
    apply_patch $patches_dir/console_bridge.patch

    # Patch urdfdom - Build as static lib
    apply_patch $patches_dir/urdfdom.patch

    # Patch qhull - Don't install shared libraries
    # TODO: Remove shared libraries to avoid hack in parse_libs.py
    # apply_patch /opt/roscpp_android/patches/qhull.patch

    # Patch bfl - Build as static lib
    apply_patch $patches_dir/bfl.patch

    # Patch orocos_kdl - Build as static lib
    apply_patch $patches_dir/orocos_kdl.patch

    # Patch PCL - Disable optionals.
    apply_patch $patches_dir/pcl-1.8.1.patch

    # Patch uuid - Avoiding stdlib.h include
    apply_patch $patches_dir/uuid.patch

    ## ROS patches

    # Patch bondcpp - Fix transitive linking problems
    apply_patch $patches_dir/bondcpp.patch

    # Patch image_publisher - Fix linking problems, transitive linking,
    # and changed shared to static library building.
    apply_patch $patches_dir/image_publisher.patch

    # Patch image_rotate - Fix find opencv and transitive linking problem
    apply_patch $patches_dir/image_rotate.patch

    # Patch opencv - Fix installation path
    apply_patch $patches_dir/opencv.patch
    
    # Patch actionlib - problems with Boost changes.
    apply_patch $patches_dir/actionlib.patch

    # Patch rospack - problems with Boost changes
    # Also emptied some unnecessary functions to avoid problems related to including Python.
    apply_patch $patches_dir/rospack.patch
    
    # Patch xmlrpcpp - problems with Boost changes.
    apply_patch $patches_dir/xmlrpcpp.patch

    # Remove
    rm -fr $OUTPUT_PREFIX/catkin_ws/src/geometry2/tf2_py

    # Patch roslib - weird issue with rospack.
    # TODO: Need to look further (only on catkin_make_isolated)
    # apply_patch /opt/roscpp_android/patches/roslib.patch

    # Patch collada_parser - cmake detects mkstemps even though Android does not support it
    # TODO: investigate how to prevent cmake to detect system mkstemps
#    apply_patch $patches_dir/collada_parser.patch

    # Patch laser_assembler - Remove testing for Android
    # TODO: It seems like there may be a better way to handle the test issues
    # http://stackoverflow.com/questions/22055741/googletest-for-android-ndk
    # apply_patch $patches_dir/laser_assembler.patch

    # Patch laser_filters - Remove testing for Android
    # TODO: It seems like there may be a better way to handle the test issues
    # http://stackoverflow.com/questions/22055741/googletest-for-android-ndk
    # https://source.android.com/reference/com/android/tradefed/testtype/GTest.html
    # apply_patch $patches_dir/laser_filters.patch

    # Patch camera_info_manager - remove testing for Android
    # TODO: It seems like there may be a better way to handle the test issues
    # http://stackoverflow.com/questions/22055741/googletest-for-android-ndk
    # https://source.android.com/reference/com/android/tradefed/testtype/GTest.html
    apply_patch $patches_dir/camera_info_manager.patch

    # Patch camera_calibration_parsers - deleted python things and solved problem finding Boost.
    apply_patch $patches_dir/camera_calibration_parsers.patch

    # Patch cv_bridge - fix transitive linking in cv_bridge-extras.cmake
    apply_patch $patches_dir/cv_bridge.patch

    # Patch robot_pose_ekf - Add bfl library cmake variables, also, remove tests
    # TODO: The correct way to handle this would be to create .cmake files for bfl and do a findpackage(orocos-bfl)
    # apply_patch $patches_dir/robot_pose_ekf.patch

    # Patch robot_state_publisher - Add ARCHIVE DESTINATION
    # TODO: Create PR to add ARCHIVE DESTINATION
    # apply_patch $patches_dir/robot_state_publisher.patch

    # Patch moveit_core - Add fcl library cmake variables
    # TODO: The correct way to handle this would be to create .cmake files for fcl and do a findpackage(fcl)
    #apply_patch $patches_dir/moveit_core.patch

    # Patch moveit_core plugins - Add ARCHIVE DESTINATION
    # TODO: PR merged: https://github.com/ros-planning/moveit_core/pull/251
    # Wait for next release to remove (current 0.6.15)
    #apply_patch $patches_dir/moveit_core_plugins.patch

    # Patch camera_calibration_parsers - Fix yaml-cpp dependency
    # TODO: PR created: https://github.com/ros-perception/image_common/pull/36
    # apply_patch $patches_dir/camera_calibration_parsers.patch

    # Patch image_view - Solved YAML linking problems, and transitive linking.
    apply_patch $patches_dir/image_view.patch

    # Patch depth_image_proc - Solved transitive linking problems
    apply_patch $patches_dir/depth_image_proc.patch

    # Patch global_planner - Add angles dependency
    # TODO: PR merged: https://github.com/ros-planning/navigation/pull/359
    # Wait for next release to remove (current 1.12.4)
    # apply_patch $patches_dir/global_planner.patch

    #Patch Poco lib
    apply_patch $patches_dir/poco.patch

    # Plugin specific patches
    if [ $use_pluginlib -ne 0 ]; then
        # Patch pluginlib for static loading
        apply_patch $patches_dir/pluginlib.patch
        # apply_patch image_transport # to fix faulty export plugins
        apply_patch $patches_dir/image_transport.patch
    fi

    ## Demo Application specific patches

fi
