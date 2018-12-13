#include <stdarg.h>
#include <stdio.h>
#include <sstream>
#include <map>
#include <string.h>
#include <errno.h>
#include <vector>
#include <set>
#include <fstream>
#include <android/log.h>

#include "ros/ros.h"
#include <std_msgs/String.h>

#include <android_native_app_glue.h>

int loop_count_ = 0;
ros::Publisher chatter_pub;

void log(const char *msg, ...) {
    va_list args;
    va_start(args, msg);
    __android_log_vprint(ANDROID_LOG_INFO, "ROSCPP_NDK_EXAMPLE", msg, args);
    va_end(args);
}

#define LASTERR strerror(errno)

void chatterCallback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("%s", msg->data.c_str());
    loop_count_++;
    std_msgs::String msgo;
    std::stringstream ss;
    ss << "hello world from android ndk " << loop_count_;
    msgo.data = ss.str();
    chatter_pub.publish(msgo);
    log(msg->data.c_str());
}

void android_main(android_app *state) {

    int argc = 3;

    //*********************** NOTE: HARDCODE rosmaster ip addresses in __master, and hardcode the ip address of the cellphone in __ip *************************
    char* argv[] = {const_cast<char*>("nothing_important") , const_cast<char*>("__master:=http://10.34.0.120:11311"), const_cast<char*>("__ip:=10.34.0.121")};
    //*********************************************************************************************************************************************************

    for(int i = 0; i < argc; i++){
        log(argv[i]);
    }

    ros::init(argc, &argv[0], "android_ndk_native_cpp");

    log("GOING TO NODEHANDLE");
    std::string master_uri = ros::master::getURI();

    if(ros::master::check()){
        log("ROS MASTER IS UP!");
    } else {
        log("NO ROS MASTER.");
    }
    log(master_uri.c_str());

    ros::NodeHandle n;

    log("GOING TO PUBLISHER");

    // Creating a publisher and a subscriber
    // When something is received in chatter topic, a message is published in a_chatter topic
    chatter_pub = n.advertise<std_msgs::String>("a_chatter", 1000);
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

    // Rate 1Hz
    ros::WallRate loop_rate(1);

    while(1) {
        int events;
        struct android_poll_source* source;

        // Poll android events, without locking
        while (ALooper_pollAll(0, NULL, &events, (void**)&source) >= 0)
        {
            // Process this event
            if (source != NULL) {
                source->process(state, source);
            }

            // Check if we are exiting.
            if (state->destroyRequested != 0) {
                log("APP DESTROYED BYE BYE");
                return;
            }
        }

        ros::spinOnce();

        if(!ros::ok()){
            log("ROS ISN'T OK, BYE BYE");
            return;
        }

        loop_rate.sleep();
    }
}
