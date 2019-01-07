#!/bin/bash
# Applies all the patch files in a given directory over an output directory.
# The output directory can be specified as parameter; otherwise OUTPUT_DIR environment variable is used.
# Required environment variables:
# - SCRIPT_DIR: where utility scripts are located.

# Abort script on any failures
set -e

source $SCRIPT_DIR/utils.sh

if [ $# < 1 ] || [ $# > 2 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 patch_prefix [output_prefix] # Will use OUTPUT_DIR environment variable as default"
    echo "  example: $0 /home/user/ros_android/patches [/home/user/ros_android/output]"
    exit 1
fi

patch_prefix=$1
output_prefix={$2:-$OUTPUT_DIR}

echo
echo -e '\e[34mApplying patches.\e[39m'
echo

for patch_file in $patch_prefix/*.patch; do
	apply_patch $patch_file -d $output_prefix
done
