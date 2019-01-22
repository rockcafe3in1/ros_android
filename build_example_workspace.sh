
CMAKE_BUILD_TYPE=Release
VERBOSE=""
ANDROID_ABI=arm64-v8a
ANDROID_STL=c++_shared       # or c++_static, see https://developer.android.com/ndk/guides/cpp-support
ANDROID_PLATFORM=android-24
RBA_TOOLCHAIN=$ANDROID_NDK_HOME/build/cmake/android.toolchain.cmake

python=$(which python)
python_lib=/usr/lib/x86_64-linux-gnu/libpython2.7.so
python_inc=/usr/include/python2.7
python2_inc=/usr/include/x86_64-linux-gnu/python2.7
prefix=$(cd output && pwd)
TARGET=$prefix/target
export ROS_PARALLEL_JOBS="-j$PARALLEL_JOBS -l$PARALLEL_JOBS"

cd example_workspace

catkin config \
  --no-extend \
  --install-space $TARGET \
  --install \
  --isolate-devel \
  --cmake-args \
    -DCMAKE_TOOLCHAIN_FILE=$RBA_TOOLCHAIN \
    -DUSE_CATKIN=ON -DCMAKE_TOOLCHAIN_FILE=$RBA_TOOLCHAIN \
    -DANDROID_ABI=${ANDROID_ABI} -DANDROID_PLATFORM=${ANDROID_PLATFORM} -DANDROID_STL=${ANDROID_STL} \
    -DPYTHON_EXECUTABLE=$python -DPYTHON_LIBRARY=$python_lib \
    -DPYTHON_INCLUDE_DIR=$python_inc -DPYTHON_INCLUDE_DIR2=$python2_inc \
    -DBUILD_SHARED_LIBS=0 -DCMAKE_INSTALL_PREFIX=$TARGET \
    -DBoost_NO_BOOST_CMAKE=ON -DBOOST_ROOT=$TARGET -DANDROID=TRUE \
    -DBOOST_INCLUDEDIR=$TARGET/include/boost -DBOOST_LIBRARYDIR=$TARGET/lib \
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} \
    -DCMAKE_FIND_ROOT_PATH=$prefix -DTARGET=$TARGET \
    -DBUILD_TESTING=OFF -DCATKIN_ENABLE_TESTING=OFF

catkin build