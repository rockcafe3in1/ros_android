prefix=$1
echo_title 'Building library dependencies.'

# if the library doesn't exist, then build it
[ -f $prefix/target/lib/libbz2.a ] || run_cmd build_library bzip2 $prefix/libs/bzip2
[ -f $prefix/target/lib/libuuid.a ] || run_cmd build_library uuid $prefix/libs/uuid
[ -f $prefix/target/lib/libboost_system.a ] || run_cmd copy_boost $prefix/libs/boost
[ -f $prefix/target/lib/libPocoFoundation.a ] || run_cmd build_library_with_toolchain poco $prefix/libs/poco-1.8.0
[ -f $prefix/target/lib/libtinyxml.a ] || run_cmd build_library tinyxml $prefix/libs/tinyxml
[ -f $prefix/target/lib/libtinyxml2.a ] || run_cmd build_library tinyxml2 $prefix/libs/tinyxml2
[ -f $prefix/target/lib/libconsole_bridge.a ] || run_cmd build_library console_bridge $prefix/libs/console_bridge
[ -f $prefix/target/lib/liblz4.a ] || run_cmd build_library lz4 $prefix/libs/lz4-r124/cmake_unofficial
[ -f $prefix/target/lib/libcurl.a ] || run_cmd build_library_with_toolchain curl $prefix/libs/curl-7.47.0
[ -f $prefix/target/include/urdf_model/model.h ] || run_cmd build_library urdfdom_headers $prefix/libs/urdfdom_headers
[ -f $prefix/target/lib/liburdfdom_model.a ] || run_cmd build_library urdfdom $prefix/libs/urdfdom
[ -f $prefix/target/lib/libiconv.a ] || run_cmd build_library_with_toolchain libiconv $prefix/libs/libiconv-1.15
[ -f $prefix/target/lib/libxml2.a ] || run_cmd build_library_with_toolchain libxml2 $prefix/libs/libxml2-2.9.7
[ -f $prefix/target/lib/libcollada-dom2.4-dp.a ] || run_cmd build_library collada_dom $prefix/libs/collada_dom
[ -f $prefix/target/lib/libassimp.a ] || run_cmd build_library assimp $prefix/libs/assimp-3.1.1
[ -f $prefix/target/include/eigen3/signature_of_eigen3_matrix_library ] || run_cmd build_library eigen $prefix/libs/eigen-3.3.5
[ -f $prefix/target/lib/libqhullstatic.a ] || run_cmd build_library qhull $prefix/libs/qhull-2015.2
[ -f $prefix/target/lib/libyaml-cpp.a ] || run_cmd build_library yaml-cpp $prefix/libs/yaml-cpp-yaml-cpp-0.6.2
[ -f $prefix/target/lib/libflann_cpp_s.a ] || run_cmd build_library flann $prefix/libs/flann
[ -f $prefix/target/lib/libpcl_common.a ] || run_cmd build_library pcl $prefix/libs/pcl-pcl-1.8.1
