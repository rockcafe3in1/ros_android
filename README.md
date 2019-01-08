# ROS for Android

These scripts will help you build static libraries
for ROS kinetic for android and setup a sample application.

You will need android SDK installed and the 'android' program
location in the $PATH.

## Installation

Build docker image and run it:
    
    docker/build.sh
    docker/run.sh

The `install.sh` script will call all the other scripts
sequentially, you just have to give it a prefix path:

    ./install.sh /path/to/workspace

## Building app samples

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
