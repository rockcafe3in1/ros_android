#!/usr/bin/env bash

SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"
OUTPUT_DIR=$SCRIPTPATH/../
IMAGE=android_ndk

if [ "$#" == 0 ]; then
  EXTRA_ARGS="bash"
else
  EXTRA_ARGS="bash -c \"$@\""
fi

#mkdir -p $OUTPUT_DIR
docker run \
  -v ${OUTPUT_DIR}:/opt/ros_android \
  --privileged \
  -it \
  ${IMAGE} "${EXTRA_ARGS}"
