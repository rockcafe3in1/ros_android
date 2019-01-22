#!/bin/bash

# Abort script on any failures
set -e

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

if [ $# != 2 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 patch_prefix output_prefix"
    echo "  example: $0 /home/user/ros_android/patches /home/user/my_workspace/output"
    exit 1
fi

patch_prefix=$1
output_prefix=$2

echo
echo -e '\e[34mApplying patches.\e[39m'
echo

for patch_file in $patch_prefix/*.patch; do
	apply_patch $patch_file -d $output_prefix
done
