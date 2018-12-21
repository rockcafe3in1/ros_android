#!/bin/bash

# Abort script on any failures
set -e

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

if [ $# != 1 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 prefix_path"
    echo "  example: $0 /home/user/my_workspace"
    exit 1
fi

prefix=$(cd $1 && pwd)

echo
echo -e '\e[34mCopying boost.\e[39m'
echo

[ "$TARGET_DIR" = "" ] && die 'could not find target basedir. Please set $TARGET_DIR environment variable.'
mkdir -p $TARGET_DIR/lib
cd $prefix/build/out/arm64-v8a/lib/
for i in *.a # Rename and move libraries (remove the gcc type, so on)
do
    #mv "$i" "`echo $i | sed 's/000//'`"
    #cp lib/lib*.a ./
    cp "$i" $TARGET_DIR/lib/"`echo $i | sed 's/ *\-.*//'`.a"
done

cd ../include
mkdir -p $TARGET_DIR/include
cp -R boost-1_68/boost $TARGET_DIR/include/
