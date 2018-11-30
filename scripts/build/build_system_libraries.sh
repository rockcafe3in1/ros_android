# This script uses the following environment variables:
# - OUTPUT_PREFIX: where the ROS workspace shall be created.
# - UTIL_DIR: Directory where basic utilities are located.

source $UTIL_DIR/basic_utils.sh
echo_title 'Building library dependencies.'

# if the library doesn't exist, then build it
[ -f $OUTPUT_PREFIX/target/lib/libbz2.a ] || run_cmd build_library bzip2 $OUTPUT_PREFIX/libs/bzip2
[ -f $OUTPUT_PREFIX/target/lib/libuuid.a ] || run_cmd build_library uuid $OUTPUT_PREFIX/libs/uuid
[ -f $OUTPUT_PREFIX/target/lib/libboost_system.a ] || run_cmd copy_boost $OUTPUT_PREFIX/libs/boost
[ -f $OUTPUT_PREFIX/target/lib/libPocoFoundation.a ] || run_cmd build_library_with_toolchain poco $OUTPUT_PREFIX/libs/poco-1.8.0
[ -f $OUTPUT_PREFIX/target/lib/libtinyxml.a ] || run_cmd build_library tinyxml $OUTPUT_PREFIX/libs/tinyxml
[ -f $OUTPUT_PREFIX/target/lib/libtinyxml2.a ] || run_cmd build_library tinyxml2 $OUTPUT_PREFIX/libs/tinyxml2
[ -f $OUTPUT_PREFIX/target/lib/libconsole_bridge.a ] || run_cmd build_library console_bridge $OUTPUT_PREFIX/libs/console_bridge
[ -f $OUTPUT_PREFIX/target/lib/liblz4.a ] || run_cmd build_library lz4 $OUTPUT_PREFIX/libs/lz4-r124/cmake_unofficial
[ -f $OUTPUT_PREFIX/target/lib/libcurl.a ] || run_cmd build_library_with_toolchain curl $OUTPUT_PREFIX/libs/curl-7.47.0
[ -f $OUTPUT_PREFIX/target/include/urdf_model/model.h ] || run_cmd build_library urdfdom_headers $OUTPUT_PREFIX/libs/urdfdom_headers
[ -f $OUTPUT_PREFIX/target/lib/liburdfdom_model.a ] || run_cmd build_library urdfdom $OUTPUT_PREFIX/libs/urdfdom
[ -f $OUTPUT_PREFIX/target/lib/libiconv.a ] || run_cmd build_library_with_toolchain libiconv $OUTPUT_PREFIX/libs/libiconv-1.15
[ -f $OUTPUT_PREFIX/target/lib/libxml2.a ] || run_cmd build_library_with_toolchain libxml2 $OUTPUT_PREFIX/libs/libxml2-2.9.7
[ -f $OUTPUT_PREFIX/target/lib/libcollada-dom2.4-dp.a ] || run_cmd build_library collada_dom $OUTPUT_PREFIX/libs/collada_dom
[ -f $OUTPUT_PREFIX/target/lib/libassimp.a ] || run_cmd build_library assimp $OUTPUT_PREFIX/libs/assimp-3.1.1
[ -f $OUTPUT_PREFIX/target/include/eigen3/signature_of_eigen3_matrix_library ] || run_cmd build_library eigen $OUTPUT_PREFIX/libs/eigen-3.3.5
[ -f $OUTPUT_PREFIX/target/lib/libqhullstatic.a ] || run_cmd build_library qhull $OUTPUT_PREFIX/libs/qhull-2015.2
[ -f $OUTPUT_PREFIX/target/lib/libyaml-cpp.a ] || run_cmd build_library yaml-cpp $OUTPUT_PREFIX/libs/yaml-cpp-yaml-cpp-0.6.2
[ -f $OUTPUT_PREFIX/target/lib/libflann_cpp_s.a ] || run_cmd build_library flann $OUTPUT_PREFIX/libs/flann
[ -f $OUTPUT_PREFIX/target/lib/libpcl_common.a ] || run_cmd build_library pcl $OUTPUT_PREFIX/libs/pcl-pcl-1.8.1
