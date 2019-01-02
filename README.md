These scripts will help you build static libraries
for ROS kinetic for android and setup a sample application.

You will need android SDK installed and the 'android' program
location in the $PATH.

INSTALL
-------

Build docker image and run it:
    
    docker/build.sh
    docker/run.sh

The `do_everything.sh` script will call all the other scripts
sequentially, you just have to give it a prefix path:

    ./do_everything.sh /path/to/workspace

BUILD SAMPLES
-------

If you also want to build the samples, use:

    ./do_everything.sh /path/to/workspace --samples

You can find the resulting apks inside `/path/to/workspace/target/apks/name_of_the_sample_app/apk_file`.
Specific instructions about how to use the samples are located inside: `files/name_of_the_sample_app/README.md`