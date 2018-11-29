echo_title 'Getting library dependencies.'
# prefix=$1

mkdir -p $prefix/libs

# Start with catkin since we use it to build almost everything else
[ -d $prefix/target ] || mkdir -p $prefix/target

# Get the android ndk build helper script
# If file doesn't exist, then download and patch it
#if ! [ -e $prefix/android.toolchain.cmake ]; then
#    cd $prefix
#    download 'https://raw.githubusercontent.com/taka-no-me/android-cmake/556cc14296c226f753a3778d99d8b60778b7df4f/android.toolchain.cmake'
#    patch -p0 -N -d $prefix < $my_loc/patches/android.toolchain.cmake.patch
#    cat $my_loc/files/android.toolchain.cmake.addendum >> $prefix/android.toolchain.cmake
#fi


# Now get boost with a specialized build
[ -d $prefix/libs/boost ] || run_cmd get_library boost $prefix/libs
[ -d $prefix/libs/bzip2 ] || run_cmd get_library bzip2 $prefix/libs
[ -d $prefix/libs/uuid ] || run_cmd get_library uuid $prefix/libs
[ -d $prefix/libs/poco-1.8.0 ] || run_cmd get_library poco $prefix/libs
[ -d $prefix/libs/tinyxml ] || run_cmd get_library tinyxml $prefix/libs
[ -d $prefix/libs/tinyxml2 ] || run_cmd get_library tinyxml2 $prefix/libs
[ -d $prefix/libs/console_bridge ] || run_cmd get_library console_bridge $prefix/libs
[ -d $prefix/libs/lz4-r124 ] || run_cmd get_library lz4 $prefix/libs
[ -d $prefix/libs/curl-7.47.0 ] || run_cmd get_library curl $prefix/libs
[ -d $prefix/libs/urdfdom/ ] || run_cmd get_library urdfdom $prefix/libs
[ -d $prefix/libs/urdfdom_headers ] || run_cmd get_library urdfdom_headers $prefix/libs
[ -d $prefix/libs/libiconv-1.15 ] || run_cmd get_library libiconv $prefix/libs
[ -d $prefix/libs/libxml2-2.9.7 ] || run_cmd get_library libxml2 $prefix/libs
[ -d $prefix/libs/collada_dom ] || run_cmd get_library collada_dom $prefix/libs
[ -d $prefix/libs/eigen-3.3.5 ] || run_cmd get_library eigen $prefix/libs
[ -d $prefix/libs/assimp-3.1.1 ] || run_cmd get_library assimp $prefix/libs
[ -d $prefix/libs/qhull-2015.2 ] || run_cmd get_library qhull $prefix/libs
[ -d $prefix/libs/yaml-cpp-yaml-cpp-0.6.2 ] || run_cmd get_library yaml-cpp $prefix/libs
[ -d $prefix/libs/flann ] || run_cmd get_library flann $prefix/libs
[ -d $prefix/libs/pcl-pcl-1.8.1 ] || run_cmd get_library pcl $prefix/libs
# get rospkg dependency for pluginlib support at build time
[ -d $my_loc/files/rospkg ] || run_cmd get_library rospkg $my_loc/files

