cmd_exists() {
    command -v $1 > /dev/null 2>&1
}

die() {
    echo $1
    exit 1
}

cmake_build() {
    cmd_exists cmake || die 'cmake was not found'

    [ "$TARGET_DIR" = "" ] && die 'could not find target basedir. Please set $TARGET_DIR environment variable.'
    [ "$RBA_TOOLCHAIN" = "" ] && die 'could not find android.toolchain.cmake, you should set RBA_TOOLCHAIN variable.'

    target=$TARGET_DIR
    python=$(which python)

    cd $1
    mkdir -p build && cd build
    cmake .. -DCMAKE_TOOLCHAIN_FILE=$RBA_TOOLCHAIN \
        -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE \
        -DANDROID_ABI=${ANDROID_ABI} -DANDROID_PLATFORM=${ANDROID_PLATFORM} -DANDROID_STL=${ANDROID_STL} \
        -DPYTHON_EXECUTABLE=$python -DCMAKE_INSTALL_PREFIX=$target -DBUILD_SHARED_LIBS=0 -DPCL_SHARED_LIBS=FALSE \
        -DCMAKE_FIND_ROOT_PATH=$target \
        -DBUILD_TESTING=OFF
    make -j$PARALLEL_JOBS -l$PARALLEL_JOBS install
}

# Check if patch hasn't already applied and apply it
apply_patch() {
    echo "Checking patch: $1"
    patch=$1
    shift
    if [ "$#" -eq 0 ]; then
        set -- -d $prefix
    fi

    set -e
    # If the reverse patch could not be applied, then the patch has to be applied.
    if ! patch -p0 -N --dry-run --silent -R "$@" < $patch ; then
        # The following call will abort the script on error.
        patch -p0 -N "$@" < $patch
    fi
}
