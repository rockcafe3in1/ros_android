#!/bin/bash

# Abort script on any failures
set -e

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

if [ $# != 2 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 library_name library_prefix_path"
    echo "  example: $0 tinyxml /home/user/my_workspace/tinyxml"
    exit 1
fi

echo
echo -e '\e[34mGetting '$1'.\e[39m'
echo

prefix=$(cd $2 && pwd)

if [ $1 == 'assimp' ]; then
    URL=https://github.com/assimp/assimp/archive/v3.1.1.tar.gz
    COMP='gz'
elif [ $1 == 'boost' ]; then
    URL=https://github.com/moritz-wundke/Boost-for-Android.git
    COMP='git'
elif [ $1 == 'bzip2' ]; then
    URL=https://github.com/osrf/bzip2_cmake.git
    COMP='git'
elif [ $1 == 'collada_dom' ]; then
    URL=https://github.com/rdiankov/collada-dom.git
    COMP='git'
    HASH='v2.4.4'
elif [ $1 == 'console_bridge' ]; then
    URL=https://github.com/ros/console_bridge.git
    COMP='git'
    HASH='0.3.2'
elif [ $1 == 'curl' ]; then
    URL=http://curl.haxx.se/download/curl-7.47.0.tar.bz2
    COMP='bz2'
elif [ $1 == 'eigen' ]; then
    URL=https://bitbucket.org/eigen/eigen/get/3.3.5.tar.gz
    COMP='gz'
elif [ $1 == 'flann' ]; then
    URL=https://github.com/chadrockey/flann_cmake.git
    COMP='git'
elif [ $1 == 'libiconv' ]; then
    URL=http://ftp.gnu.org/pub/gnu/libiconv/libiconv-1.15.tar.gz
    COMP='gz'
elif [ $1 == 'libxml2' ]; then
    URL=ftp://xmlsoft.org/libxml2/libxml2-2.9.7.tar.gz
    COMP='gz'
elif [ $1 == 'lz4' ]; then
    URL=https://github.com/Cyan4973/lz4/archive/r124.tar.gz
    COMP='gz'
elif [ $1 == 'pcl' ]; then
    URL=https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.1.tar.gz
    COMP='gz'
elif [ $1 == 'poco' ]; then
    URL=http://pocoproject.org/releases/poco-1.8.0/poco-1.8.0.tar.gz
    COMP='gz'
elif [ $1 == 'qhull' ]; then
    URL=http://www.qhull.org/download/qhull-2015-src-7.2.0.tgz
    COMP='gz'
elif [ $1 == 'tinyxml' ]; then
    URL=https://github.com/chadrockey/tinyxml_cmake
    COMP='git'
elif [ $1   == 'tinyxml2' ]; then
    URL=https://github.com/leethomason/tinyxml2
    COMP='git'
elif [ $1 == 'urdfdom_headers' ]; then
    URL=https://github.com/ros/urdfdom_headers.git
    COMP='git'
    HASH='0.4.2'
elif [ $1 == 'urdfdom' ]; then
    URL=https://github.com/ros/urdfdom.git
    COMP='git'
    HASH='0.4.2'
elif [ $1 == 'uuid' ]; then
    URL=https://github.com/chadrockey/uuid_cmake
    COMP='git'
elif [ $1 == 'yaml-cpp' ]; then
    URL=https://github.com/jbeder/yaml-cpp/archive/yaml-cpp-0.6.2.tar.gz
    COMP='gz'
elif [ $1 == 'rospkg' ]; then
    URL=https://github.com/ros-infrastructure/rospkg.git
    COMP='git'
    HASH='93b1b72f256badf22ccc926b22646f2e83b720fd'
fi

if [ $COMP == 'gz' ]; then
    download_gz $URL $prefix
elif [ $COMP == 'bz2' ]; then
    download_bz2 $URL $prefix
elif [ $COMP == 'git' ];then
    git clone $URL $prefix/$1
fi

if [ $1 == 'boost' ]; then
    cd $prefix/boost
    bash -x ./build-android.sh $ANDROID_NDK --boost=1.68.0 --arch=$ANDROID_ABI
elif [ -v HASH ]; then
    cd $prefix/$1
    git checkout $HASH
elif [ $1 == 'eigen' ]; then
    mv $prefix/eigen-eigen-* $prefix/eigen-3.3.5
fi
