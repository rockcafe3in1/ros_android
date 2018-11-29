cmd_exists() {
    command -v $1 > /dev/null 2>&1
}

die() {
    echo $1
    exit 1
}

download() {
    if [ ! -z $1 ]; then
        cmd_exists curl && curl -L $1 -O || wget $1
    else 
        echo "skipping download of $1 as it's already local"
    fi
}

download_bz2() {
    echo "downloading $1"
    ( cmd_exists curl && curl -L $1 || wget -O - $1 ) | tar jx -C $2
}

download_gz() {
    echo "downloading $1"
    ( cmd_exists curl && curl -L $1 || wget -O - $1 ) | tar zx -C $2
}

download_zip() {
    cmd_exists unzip || die 'could not find unzip'

    echo "downloading $1"

    tmpdir=$(mktemp -d /tmp/rba.XXXX)
    tmpfile=$tmpdir/gtest.zip
    ( cmd_exists curl && curl -L $1 -o $tmpfile || wget $1 -O $tmpfile ) && unzip $tmpfile -d $2
    rm -rf $tmpdir
}

cmake_build() {
    cmd_exists cmake || die 'cmake was not found'

    [ "$CMAKE_PREFIX_PATH" = "" ] && die 'could not find target basedir. Have you run build_catkin.sh and sourced setup.bash?'
    [ "$RBA_TOOLCHAIN" = "" ] && die 'could not find android.toolchain.cmake, you should set RBA_TOOLCHAIN variable.'

    target=$CMAKE_PREFIX_PATH
    python=$(which python)

    cd $1
    mkdir -p build && cd build
    cmake .. -DCMAKE_TOOLCHAIN_FILE=$RBA_TOOLCHAIN \
        -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE \
        -DANDROID_ABI=arm64-v8a -DANDROID_NATIVE_API_LEVEL=$platform \
        -DPYTHON_EXECUTABLE=$python -DCMAKE_INSTALL_PREFIX=$target -DBUILD_SHARED_LIBS=0 -DPCL_SHARED_LIBS=FALSE \
        -DCMAKE_FIND_ROOT_PATH=$target \
        -DBUILD_TESTING=OFF
    make -j$PARALLEL_JOBS -l$PARALLEL_JOBS install
}

# Check if patch hasn't already applied and apply it
apply_patch() {
    echo "Checking patch: $1"
    if patch -p0 -N --dry-run --silent -d $prefix < $1;
    then
        patch -p0 -N -d $prefix < $1 || return $?
    fi
    echo ''
}
