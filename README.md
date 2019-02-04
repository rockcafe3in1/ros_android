# ROS for Android

These scripts will help you build static libraries
for ROS kinetic for android and setup sample applications.

## Quick installation with Docker

Simply run `dockerized_install.sh` script. It will setup a Docker image, download packages and system dependencies, and cross compile everything using Catkin.
The result workspace with the code and the compiled libraries will be placed in `/path/to/ros_android/output`.

You can use the sample applications as a guide to build an Android app on top of the cross compiled libraries using Catkin, Gradle and CMake.

## [Installation - under the hood](#installation)

Build docker image and run it:
    
    docker/build.sh
    docker/run.sh

The `install.sh` script will call all the other scripts
sequentially, you just have to give it a prefix path:

    ./install.sh /path/to/workspace

### Building app samples

If you also want to build the samples, use:

    ./install.sh /path/to/workspace --samples

You can find the resulting apks inside `/path/to/workspace/target/apks/name_of_the_sample_app/apk_file`.
Specific instructions about how to use the samples are located inside: `files/name_of_the_sample_app/README.md`

## Adding new packages or dependencies

Adding new packages may not be as straightforward as adding a new line in a list.
There are multiple sources of problems when cross compiling. In some cases, `CMake` scripts are not ready for
cross compilation as they tend to look for dependencies in standard paths, or they don't properly expose transitive dependencies. 
In some other cases, the codebases use dependencies that are not available on Android, or it just doesn't make sense to use them
on Android without modifications (e.g. `pluginlib`).

The following steps should serve as a guide when adding new packages or dependencies:

- Add the package to `ros.rosinstall` or `system_deps.rosinstall`; use existing packages as an example.
- Add a patch if necessary to `patches` directory.
- If it's a system dependency, add a build rule to `install.sh`. According to the library, it may be necessary to use `build_library` script or adding a special rule to `build_library_with_toolchain`.
- That's it! Test your build and watch out build errors. Using `verbose` flags when building should help finding hints about the problems that may arise.

## Building your own apps and libraries on top of ROS cross-compiled workspace

You can use `build_catkin_workspace` as a standalone script to build your Android application or library as a catkin package,
making use of the ROS cross compiled libraries and system dependencies. The examples under [example_workspace](https://github.com/Intermodalics/ros_android/tree/kinetic/example_workspace/src) can be used as a starting point and as a reference to build your code; in short they are Gradle projects wrapped as Catkin packages.

Here's some guidelines to take into account when creating new projects, using `hello_world` app as an example:
- Your code should have the structure of a ROS Android package, with a top level `CMakeLists` to make Gradle interact with Catkin. Take [this CMakeLists file](https://github.com/Intermodalics/ros_android/blob/kinetic/example_workspace/src/hello_world_example_app/CMakeLists.txt) as an example.
- A [package.xml](https://github.com/Intermodalics/ros_android/blob/kinetic/example_workspace/src/hello_world_example_app/package.xml) has to be created, declaring Catkin as the build tool, and the ROS dependencies.
- In the [bottom level gradle buildscript](https://github.com/Intermodalics/ros_android/blob/kinetic/example_workspace/src/hello_world_example_app/app/build.gradle), add the necessary arguments to the `externalNativeBuild` block, and specify ABI filters as in [here](https://github.com/Intermodalics/ros_android/blob/kinetic/example_workspace/src/hello_world_example_app/app/build.gradle#L10-L23). Then, specify the path to the bottom level CMakeLists file as in [here](https://github.com/Intermodalics/ros_android/blob/kinetic/example_workspace/src/hello_world_example_app/app/build.gradle#L31-L36).
- Finally, write your [bottom level CMakeLists file](https://github.com/Intermodalics/ros_android/blob/kinetic/example_workspace/src/hello_world_example_app/app/src/main/cpp/CMakeLists.txt). Note that the way of finding and adding packages as dependencies should be just as in any regular Catkin project.

Once your project is ready to be built, place it inside a catkin workspace (i.e. `/path/to/your/workspace/src`). Then, it can be built with the standalone script like this:
```
build_catkin_workspace.sh -w /path/to/your/workspace -p /path/to/install_space -e /path/to/workspace -b Debug -v 1
```

where `/path/to/workspace` is the root directory of the cross compiled packages and libraries in the [installation step](#installation). Debug and verbose flags are recommended while developing.