#!/usr/bin/env bash

SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"
REPO_ROOT=$SCRIPTPATH/../
IMAGE=android_ndk

docker run \
  -v ${REPO_ROOT}:/opt/ros_android \
  --privileged \
  -it \
  "$@" \
  ${IMAGE} bash
