# android_ndk docker image

This image contains all the basic tools required to cross compile the ROS packages and its dependencies.

To build the image, run `build.sh`. To run the image, run `run.sh`. `ros_android` folder and all its contents will be mapped to `/opt/ros_android` inside the docker container. 

Once inside the container, place the output in a child directory of `/opt/ros_android` to be able to access it from your host computer.
