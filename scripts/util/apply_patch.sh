#!/bin/bash

# Check if patch hasn't already applied and apply it
apply_patch() {
    set -e
    echo "Checking patch: $1"
    if patch -p0 -N --dry-run --silent -d $OUTPUT_PREFIX < $1;
    then
        patch -p0 -N -d $OUTPUT_PREFIX < $1 || return $?
    fi
    echo ''
}
