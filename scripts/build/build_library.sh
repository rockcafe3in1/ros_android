#!/bin/bash
# This script uses the following environment variables:
# - UTIL_DIR: Directory where basic utilities are located.

# Abort script on any failures
set -e

if [ $# != 2 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 libary library_source_dir"
    echo "  example: $0 tinyxml /home/user/my_workspace/tinyxml"
    exit 1
fi

# Source required scripts
source $UTIL_DIR/basic_utils.sh
source $UTIL_DIR/cmake_build.sh

echo_title 'Building '$1'.'

cmake_build $2

if [ $1 == 'eigen' ]; then
	cp -r $CMAKE_PREFIX_PATH/include/eigen3/* $CMAKE_PREFIX_PATH/include
fi

# TODO(ivanpauno): Check this later.
# if [ $1 == 'opencv' ]; then
#     echo "Copy opencv 3rdparty libraries to the lib folder."
#     echo "These are needed to build the compressed image transport plugin."
#     (cp $2/build/3rdparty/lib/* $2/../../target/lib/)
# fi

