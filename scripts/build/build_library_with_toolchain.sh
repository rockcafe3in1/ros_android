#!/bin/bash
# This script uses the following environment variables:
# - ANDROID_NDK: Android NDK's directory.
# - CMAKE_PREFIX_PATH: CMake's prefix path.
# - PARALLEL_JOBS: how many threads to use to build the library.
# - UTIL_DIR: Directory where basic utilities are located.
# Parameters:
# - current_lib_prefix: directory for the library to build.

# Abort script on any failures
set -e

if [ $# != 2 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 library_name library_source_dir"
    echo "  example: $0 /home/user/my_workspace/libxml2-2.9.1"
    exit 1
fi

# Source required scripts
source $UTIL_DIR/basic_utils.sh

current_lib_prefix=$(cd $2 && pwd)
pushd $current_lib_prefix

# Create a stand alone version of the android toolchain
echo_title "Building $1."

[ "$CMAKE_PREFIX_PATH" = "" ] && die 'could not find target basedir. Have you run build_catkin.sh and sourced setup.bash?'

if [ ! -d toolchain/ ]; then
  $ANDROID_NDK/build/tools/make-standalone-toolchain.sh --install-dir=./toolchain --arch=$arch
fi
export PATH=$PATH:$2/toolchain/bin

# Set --host: The system where built programs and libraries will run.
# (https://www.gnu.org/software/automake/manual/html_node/Cross_002dCompilation.html)
build=`uname -m`-linux
host=$(basename $current_lib_prefix/toolchain/*-linux-android)

# General options to pass to ./configure script
configure_options="--prefix=$CMAKE_PREFIX_PATH --disable-shared --enable-static --build=${build} --host=${host}"

# Overwrite/extend for specific packages
if [ $1 == 'poco' ]; then
    configure_options="--config=Android_static --no-samples --no-tests"
    export ANDROID_ABI=$abi
elif [ $1 == 'curl' ]; then
    configure_options="$configure_options --without-ssl --disable-tftp --disable-sspi --disable-ipv6 --disable-ldaps --disable-ldap --disable-telnet --disable-pop3 --disable-ftp --disable-imap --disable-smtp --disable-pop3 --disable-rtsp --disable-ares --without-ca-bundle --disable-warnings --disable-manual --without-nss --without-random"
elif [ $1 == 'libxml2' ]; then
    configure_options="$configure_options --without-python"
fi

# Configure and build
./configure ${configure_options}
make -j$PARALLEL_JOBS -l$PARALLEL_JOBS V=1

# Install
if [ $1 == 'poco' ]; then
    mkdir -p $CMAKE_PREFIX_PATH/lib
    cp $current_lib_prefix/lib/Android/$abi/lib*.a $CMAKE_PREFIX_PATH/lib
    mkdir -p $CMAKE_PREFIX_PATH/include
    cp -r $current_lib_prefix/Foundation/include/Poco $CMAKE_PREFIX_PATH/include
else
    make install
fi

popd # $current_lib_prefix