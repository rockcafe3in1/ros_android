#
# Copyright (C) The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

cmake_minimum_required(VERSION 3.4.1)
project(image_transport_sample)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs rosconsole image_transport compressed_image_transport theora_image_transport)

include_directories(${catkin_INCLUDE_DIRS})

# build native_app_glue as a static lib
set(${CMAKE_C_FLAGS}, "${CMAKE_C_FLAGS}")
add_library(native_app_glue STATIC
    ${ANDROID_NDK}/sources/android/native_app_glue/android_native_app_glue.c)

# now build app's shared lib
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -Wall -Werror")

# Export ANativeActivity_onCreate(),
# Refer to: https://github.com/android-ndk/ndk/issues/381.
set(CMAKE_SHARED_LINKER_FLAGS
    "${CMAKE_SHARED_LINKER_FLAGS} -u ANativeActivity_onCreate")

add_library(${PROJECT_NAME} SHARED 
    src/main/cpp/image_transport_plugin_test.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${ANDROID_NDK}/sources/android/native_app_glue)

# This is to be sure that the symbols of this libraries aren't dropped
# Also, put them in catkin find_package, in order to solve transitive dependencies without problems.
SET (WHOLE_ARCHIVE_LIBRARIES
     -Wl,--whole-archive
     $ENV{TARGET_DIR}/lib/libimage_transport_plugins.a
     $ENV{TARGET_DIR}/lib/libtheora_image_transport.a
     $ENV{TARGET_DIR}/lib/libcompressed_image_transport.a
     -Wl,--no-whole-archive
)

# add lib dependencies
target_link_libraries(${PROJECT_NAME}
    android
    native_app_glue
    log
    ${WHOLE_ARCHIVE_LIBRARIES}
    ${catkin_LIBRARIES}
)
