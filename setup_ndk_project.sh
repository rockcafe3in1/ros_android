#!/bin/bash

# Abort script on any failures
set -e

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

if [ $# != 2 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 project_path portable"
    echo "  example: $0 /home/user/my_workspace/tf2_ndk 0, will use external links"
    echo "  example: $0 /home/user/my_workspace/tf2_ndk 1, will use copy all required files"
    exit 1
fi

[ "$TARGET_DIR" = "" ] && die 'could not find target basedir. Please set $TARGET_DIR environment variable.'

if [ ! -d $1 ]; then
    mkdir -p $1
fi

cd $1

if [[ $2 -eq 0 ]]; then
  ln -fs $TARGET_DIR/include ./
  ln -fs $TARGET_DIR/lib ./
  ln -fs $TARGET_DIR/share ./
else
  cp -r $TARGET_DIR/include ./
  cp -r $TARGET_DIR/lib ./
  cp -r $TARGET_DIR/share ./
fi

cp $my_loc/files/tfa/*.mk ./
