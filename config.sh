# Configures environment variables required for other scripts to work properly.

if [ $# != 1 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: source $0 prefix_path"
    echo "  example: source $0 tinyxml /home/user/ros_android/output"
    exit 1
fi

# Configure the Android NDK and toolchain
export ANDROID_ABI=arm64-v8a
export ANDROID_STL=c++_static       # or c++_shared, see https://developer.android.com/ndk/guides/cpp-support
export ANDROID_PLATFORM=android-24

# Enable this value for debug build
#CMAKE_BUILD_TYPE=Debug
export CMAKE_BUILD_TYPE=Release

# Enable this if you need to use pluginlib in Android.
# The plugins will be statically linked
export USE_PLUGINLIB=1

# Define the number of simultaneous jobs to trigger for the different
# tasks that allow it. Use the number of available processors in the
# system.
export PARALLEL_JOBS=$(nproc)

# Export common paths
export OUTPUT_DIR=$1
export TARGET_DIR=$OUTPUT_DIR/target
export LIBS_DIR=$OUTPUT_DIR/libs
export SCRIPT_DIR="$(cd "$(dirname $0)" && pwd)"
