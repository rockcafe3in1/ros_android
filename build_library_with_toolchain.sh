#!/bin/bash

# Abort script on any failures
set -e

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

if [ $# != 2 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 library_name library_source_dir"
    echo "  example: $0 /home/user/my_workspace/libxml2-2.9.1"
    exit 1
fi

prefix=$(cd $2 && pwd)

cd $2

# Create a stand alone version of the android toolchain
echo
echo -e '\e[34mBuilding '$1'.\e[39m'
echo

[ "$CMAKE_PREFIX_PATH" = "" ] && die 'could not find target basedir. Have you run build_catkin.sh and sourced setup.bash?'

if [ ! -d toolchain/ ]; then
  $ANDROID_NDK/build/tools/make-standalone-toolchain.sh --install-dir=./toolchain --arch=$arch
fi
export PATH=$PATH:$2/toolchain/bin

# Set --host: The system where built programs and libraries will run.
# (https://www.gnu.org/software/automake/manual/html_node/Cross_002dCompilation.html)
host=$(dirname $2/toolchain/*-linux-android)

if [ $1 == 'poco' ]; then
    ./configure --config=Android_static --no-samples --no-tests
    export ANDROID_ABI=$abi
elif [ $1 == 'curl' ]; then
    ./configure --prefix=$CMAKE_PREFIX_PATH --disable-shared --enable-static --host=${host} --without-ssl --disable-tftp --disable-sspi --disable-ipv6 --disable-ldaps --disable-ldap --disable-telnet --disable-pop3 --disable-ftp --disable-imap --disable-smtp --disable-pop3 --disable-rtsp --disable-ares --without-ca-bundle --disable-warnings --disable-manual --without-nss --without-random
else
    ./configure --prefix=$CMAKE_PREFIX_PATH --disable-shared --enable-static --host=${host}
fi

make -s -j$PARALLEL_JOBS -l$PARALLEL_JOBS

if [ $1 == 'poco' ]; then
    mkdir -p $CMAKE_PREFIX_PATH/lib
    cd $CMAKE_PREFIX_PATH/lib
    cp $prefix/lib/Android/$abi/lib*.a ./
    mkdir -p ../include && cd ../include
    cp -r $prefix/Foundation/include/Poco ./
else
    make install
fi

#if [ $1 == 'curl' ]; then
#    sed -i 's/#define CURL_SIZEOF_LONG 8/#define CURL_SIZEOF_LONG 4/g' $CMAKE_PREFIX_PATH/include/curl/curlbuild.h
#    sed -i 's/#define CURL_SIZEOF_CURL_OFF_T 8/#define CURL_SIZEOF_CURL_OFF_T 4/g' $CMAKE_PREFIX_PATH/include/curl/curlbuild.h
#fi
