#!/usr/bin/env bash

SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"
REPO_ROOT=$SCRIPTPATH/../
IMAGE=android_ndk

if [ "$#" == 0 ]; then
  EXTRA_ARGS=(bash)
else
  EXTRA_ARGS=(bash -c "$*")
fi

docker run \
  -v ${REPO_ROOT}:/opt/ros_android \
  --privileged \
  -it \
  ${IMAGE} "${EXTRA_ARGS[@]}"
