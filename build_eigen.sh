#!/bin/bash

# Abort script on any failures
set -e

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

if [ $# != 1 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 boost_source_dir"
    echo "  example: $0 /home/user/my_workspace/poco-1.4.6p2"
    exit 1
fi

prefix=$(cd $1 && pwd)

[ "$TARGET_DIR" = "" ] && die 'could not find target basedir. Please set $TARGET_DIR environment variable.'
cd $TARGET_DIR
mkdir -p include && cd include
cp -r $prefix/Eigen ./
cp -r $prefix/unsupported ./
