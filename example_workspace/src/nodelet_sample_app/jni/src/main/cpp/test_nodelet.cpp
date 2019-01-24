/**
 * (c) 2016 Ernesto Corbellini, Creativa77 SRL
 * 
 * Demonstrates the use of ROS nodelets in Android.
 * Example that loads the tutorial math nodelet. It listens in the
 * 'in' topic for a value and adds it to an initial value (default zero)
 * and outputs the result in the 'out' topic.
 * 
 **/

#ifndef ROS_MASTER_URI
#define ROS_MASTER_URI "__master:=http://10.34.0.120:11311"
#endif
#ifndef ROS_ANDROID_IP
#define ROS_ANDROID_IP "__ip:=10.34.0.121"
#endif

#include "ros/ros.h"
#include <nodelet/loader.h>
#include <iostream>
#include <string>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "ifaddrs.h"

#include <android_native_app_glue.h>
#include <android/log.h>

#define  LOG_TAG    "NODELET_TEST"

#define  LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, ##__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, ##__VA_ARGS__)

bool getHostIp(const char *interface, char *ipaddr);

void android_main(android_app *state)
{    
    bool res;
    std::map<std::string, std::string> remappings;
    std::vector<std::string> nodelet_argv;
    char strbuf[128];
    char ipAddr[20];
    const char *argv[] = {"cmd", ROS_MASTER_URI, ROS_ANDROID_IP};
    int argc = 3;
    
    // Dynamically obtain the IP of the host we are running in.
    if (!getHostIp("wlan0", ipAddr))
    {
        LOGD("Failed to get IP address for this interface!");
        return;
    }
    sprintf(strbuf, "__ip:=%s", ipAddr);
    argv[2] = strbuf;

    LOGD("Starting program...");
    
    ros::init(argc, const_cast<char**>(&argv[0]), "simple_nodelet_loader");

    std::string master_uri = ros::master::getURI();

    if (ros::master::check())
    {
        LOGD("ROS master is up at %s", master_uri.c_str());
        LOGD("Local address is %s", ipAddr);
    } else
    {
        LOGD("Failed to find ROS master!");
        ANativeActivity_finish(state->activity);
    }    
    
    ros::NodeHandle nh;

    nodelet::Loader loader(nh);

    //ros::param::set("/test_nodelet/value", 11.4);

    LOGD("Loading nodelet...");        
    res = loader.load("/test_nodelet", "nodelet_tutorial_math/Plus", remappings, nodelet_argv);
    
    if (!res)
    {
        LOGD("Problem loading nodelet!");
        return;
    }

    ros::AsyncSpinner spinner(4);
    spinner.start();

    LOGD("Starting ROS main loop...");
    
    while(1) {
        int events;
        struct android_poll_source* source;

        // Poll android events. Check whatever ros died every five seconds.
        while (ALooper_pollAll(5000, NULL, &events, (void**)&source) >= 0) {
            // Process this event
            if (source != NULL) {
                source->process(state, source);
            }

            // Check if we are exiting.
            if (state->destroyRequested != 0) {
                LOGD("APP DESTROYED BYE BYE");
                return;
            }
            LOGD("Polling");
        }
        LOGD("Timeout");
        if (!ros::ok() || !ros::master::check()) {
            LOGD("ROS ISN'T OK, BYE BYE");
            ANativeActivity_finish(state->activity);
        }
    }

    LOGD("Program ending...");
    
    return;
}

// Get the current host IP address on the specified interface.
bool getHostIp(const char *interface, char *ipaddr)
{
    struct ifaddrs *ifap, *ifa;
    struct sockaddr_in *sa;
    char *addr = NULL;

    getifaddrs(&ifap);

    for (ifa = ifap; ifa; ifa = ifa->ifa_next)
    {
        if (ifa->ifa_addr && ifa->ifa_addr->sa_family == AF_INET)
        {
            if (strcmp(ifa->ifa_name, interface) == 0)
            {
                sa = (struct sockaddr_in *) ifa->ifa_addr;
                addr = inet_ntoa(sa->sin_addr);
            }
        }
    }
    
    if (!addr) return(false);
    
    strcpy(ipaddr, addr);

    freeifaddrs(ifap);
    return(true);
}
