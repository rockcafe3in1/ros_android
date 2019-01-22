#!/usr/bin/env bash

IMAGE=android_ndk

pushd "$( dirname "${BASH_SOURCE[0]}" )"
docker build -t ${IMAGE} "$@" .
popd
