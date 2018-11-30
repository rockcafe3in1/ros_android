# This script uses the following environment variables:
# - OUTPUT_PREFIX: where the ROS workspace shall be created.
# Parameters:
# - helper_files_directory: where pluginlib helper files are located.

helper_files_dir=$1

# Before build
# Search packages that depend on pluginlib and generate plugin loader.
if [ $use_pluginlib -ne 0 ]; then
    echo_title 'Building pluginlib support'

    pluginlib_helper_file=pluginlib_helper.cpp
    $helper_files_dir/pluginlib_helper/pluginlib_helper.py -scanroot $OUTPUT_PREFIX/catkin_ws/src -cppout $helper_files_dir/pluginlib_helper/$pluginlib_helper_file
    cp $helper_files_dir/pluginlib_helper/$pluginlib_helper_file $OUTPUT_PREFIX/catkin_ws/src/pluginlib/src/
    line="add_library(pluginlib STATIC src/pluginlib_helper.cpp)"
    # temporally turn off error detection
    set +e
    grep "$line" $OUTPUT_PREFIX/catkin_ws/src/pluginlib/CMakeLists.txt
    # if line is not already added, then add it to the pluginlib cmake
    if [ $? -ne 0 ]; then
        # backup the file
        cp $OUTPUT_PREFIX/catkin_ws/src/pluginlib/CMakeLists.txt $OUTPUT_PREFIX/catkin_ws/src/pluginlib/CMakeLists.txt.bak
        sed -i '/INCLUDE_DIRS include/a LIBRARIES ${PROJECT_NAME}' $OUTPUT_PREFIX/catkin_ws/src/pluginlib/CMakeLists.txt
        echo -e "\n"$line >> $OUTPUT_PREFIX/catkin_ws/src/pluginlib/CMakeLists.txt
        echo 'install(TARGETS pluginlib RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})' >> $OUTPUT_PREFIX/catkin_ws/src/pluginlib/CMakeLists.txt
    fi
    # turn error detection back on
    set -e
fi
