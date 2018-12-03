# Configure the Android NDK and toolchain
export ANDROID_ABI=arm64-v8a
export ANDROID_STL=c++_static       # or c++_shared, see https://developer.android.com/ndk/guides/cpp-support
export ANDROID_PLATFORM=android-24

# Enable this value for debug build
#CMAKE_BUILD_TYPE=Debug
CMAKE_BUILD_TYPE=Release

# Enable this if you need to use pluginlib in Android.
# The plugins will be statically linked
use_pluginlib=1
