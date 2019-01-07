#!/bin/bash
# Builds a target library using cmake.
# 
# Required environment variables:
# - SCRIPT_DIR: where utility scripts are located.

# Abort script on any failures
set -e

source $SCRIPT_DIR/utils.sh

if [ $# != 2 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 libary library_source_dir"
    echo "  example: $0 tinyxml /home/user/ros_android/output/libs/tinyxml"
    exit 1
fi

echo
echo -e '\e[34mBuilding '$1.'\e[39m'
echo

cmake_build $2
