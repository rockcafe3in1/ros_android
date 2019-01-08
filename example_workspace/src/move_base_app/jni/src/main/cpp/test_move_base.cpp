#include "ros/ros.h"
#include <move_base/move_base.h>

#include <android_native_app_glue.h>
#include <android/log.h>

void log(const char *msg, ...) {
    va_list args;
    va_start(args, msg);
    __android_log_vprint(ANDROID_LOG_INFO, "MOVE_BASE_NDK_EXAMPLE", msg, args);
    va_end(args);
}

void android_main(android_app *state) {
    int argc = 4;
    // TODO: don't hardcode ip addresses
    const char *argv[] = {"nothing_important" , "__master:=http://10.34.0.120:11311",
                    "__ip:=10.34.0.121", "cmd_vel:=navigation_velocity_smoother/raw_cmd_vel"};

    for(int i = 0; i < argc; i++){
        log(argv[i]);
    }

    ros::init(argc, const_cast<char **>(&argv[0]), "move_base");

    std::string master_uri = ros::master::getURI();

    if(ros::master::check()){
        log("ROS MASTER IS UP!");
    } else {
        log("NO ROS MASTER.");
    }
    log(master_uri.c_str());

    ros::NodeHandle n;

    tf::TransformListener tf(ros::Duration(10));
    move_base::MoveBase move_base(tf);

    ros::WallRate loop_rate(100);
    
    while(1) {
        int events;
        struct android_poll_source* source;

        // Poll android events, without locking
        while (ALooper_pollAll(0, NULL, &events, (void**)&source) >= 0) {
            // Process this event
            if (source != NULL) {
                source->process(state, source);
            }

            // Check if we are exiting.
            if (state->destroyRequested != 0) {
                log("APP DESTROYED BYE BYE");
                return;
            }

            if (ros::ok()) {
                ros::spinOnce();
                loop_rate.sleep();
            }
        }
    }
}
