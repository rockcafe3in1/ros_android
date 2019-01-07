#!/bin/bash
# Applies all the patch files in a given directory over an output directory.
# The output directory can be specified as parameter; otherwise OUTPUT_DIR environment variable is used.
# See help for required positional arguments.
#
# Required environment variables:
# - SCRIPT_DIR: where utility scripts are located.
# - $OUTPUT_DIR: default output directory if positional argument is not set.

# Abort script on any failures
set -e

source $SCRIPT_DIR/utils.sh

if [ $# -lt 1 ] || [ $# -gt 2 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 patch_prefix [output_prefix] # Will use OUTPUT_DIR environment variable as default"
    echo "  example: $0 /home/user/ros_android/patches [/home/user/ros_android/output]"
    exit 1
fi

patch_prefix=$1
output_prefix=${2:-"$OUTPUT_DIR"}

[ "$output_prefix" = "" ] && die 'Output prefix not set. Please set OUTPUT_DIR environment variable.'

echo
echo -e '\e[34mApplying patches.\e[39m'
echo

for patch_file in $patch_prefix/*.patch; do
	apply_patch $patch_file -d $output_prefix
done
