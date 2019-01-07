#!/bin/bash
# Abort script on any failures
set -e

my_loc="$(cd "$(dirname $0)" && pwd)"
source $SCRIPT_DIR/utils.sh

$my_loc/docker/build.sh
$my_loc/docker/run.sh $my_loc/install.sh $my_loc/output
