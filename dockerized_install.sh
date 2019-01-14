#!/bin/bash
# Abort script on any failures
set -e

my_loc="$(cd "$(dirname $0)" && pwd)"

$my_loc/docker/build.sh
$my_loc/docker/run.sh /opt/ros_android/install.sh /opt/ros_android/output --samples
