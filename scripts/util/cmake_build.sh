#!/bin/bash

function cmake_build() {
	source $UTIL_DIR/basic_utils.sh
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
