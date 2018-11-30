# This script uses the following environment variables:
# - OUTPUT_PREFIX: where the libraries shall be installed.
# - ROS_ANDROID_ROOT: the root directory of the repository.
# - UTIL_DIR: Directory where basic utilities are located.

source $UTIL_DIR/basic_utils.sh
echo_title 'Getting library dependencies.'

mkdir -p $OUTPUT_PREFIX/libs

# Start with catkin since we use it to build almost everything else
[ -d $OUTPUT_PREFIX/target ] || mkdir -p $OUTPUT_PREFIX/target

# Now get boost with a specialized build
[ -d $OUTPUT_PREFIX/libs/boost ] || run_cmd get_library boost $OUTPUT_PREFIX/libs
[ -d $OUTPUT_PREFIX/libs/bzip2 ] || run_cmd get_library bzip2 $OUTPUT_PREFIX/libs
[ -d $OUTPUT_PREFIX/libs/uuid ] || run_cmd get_library uuid $OUTPUT_PREFIX/libs
[ -d $OUTPUT_PREFIX/libs/poco-1.8.0 ] || run_cmd get_library poco $OUTPUT_PREFIX/libs
[ -d $OUTPUT_PREFIX/libs/tinyxml ] || run_cmd get_library tinyxml $OUTPUT_PREFIX/libs
[ -d $OUTPUT_PREFIX/libs/tinyxml2 ] || run_cmd get_library tinyxml2 $OUTPUT_PREFIX/libs
[ -d $OUTPUT_PREFIX/libs/console_bridge ] || run_cmd get_library console_bridge $OUTPUT_PREFIX/libs
[ -d $OUTPUT_PREFIX/libs/lz4-r124 ] || run_cmd get_library lz4 $OUTPUT_PREFIX/libs
[ -d $OUTPUT_PREFIX/libs/curl-7.47.0 ] || run_cmd get_library curl $OUTPUT_PREFIX/libs
[ -d $OUTPUT_PREFIX/libs/urdfdom/ ] || run_cmd get_library urdfdom $OUTPUT_PREFIX/libs
[ -d $OUTPUT_PREFIX/libs/urdfdom_headers ] || run_cmd get_library urdfdom_headers $OUTPUT_PREFIX/libs
[ -d $OUTPUT_PREFIX/libs/libiconv-1.15 ] || run_cmd get_library libiconv $OUTPUT_PREFIX/libs
[ -d $OUTPUT_PREFIX/libs/libxml2-2.9.7 ] || run_cmd get_library libxml2 $OUTPUT_PREFIX/libs
[ -d $OUTPUT_PREFIX/libs/collada_dom ] || run_cmd get_library collada_dom $OUTPUT_PREFIX/libs
[ -d $OUTPUT_PREFIX/libs/eigen-3.3.5 ] || run_cmd get_library eigen $OUTPUT_PREFIX/libs
[ -d $OUTPUT_PREFIX/libs/assimp-3.1.1 ] || run_cmd get_library assimp $OUTPUT_PREFIX/libs
[ -d $OUTPUT_PREFIX/libs/qhull-2015.2 ] || run_cmd get_library qhull $OUTPUT_PREFIX/libs
[ -d $OUTPUT_PREFIX/libs/yaml-cpp-yaml-cpp-0.6.2 ] || run_cmd get_library yaml-cpp $OUTPUT_PREFIX/libs
[ -d $OUTPUT_PREFIX/libs/flann ] || run_cmd get_library flann $OUTPUT_PREFIX/libs
[ -d $OUTPUT_PREFIX/libs/pcl-pcl-1.8.1 ] || run_cmd get_library pcl $OUTPUT_PREFIX/libs
# get rospkg dependency for pluginlib support at build time
[ -d $ROS_ANDROID_ROOT/files/rospkg ] || run_cmd get_library rospkg $ROS_ANDROID_ROOT/files
