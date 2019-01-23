#include <stdarg.h>
#include <sstream>

#include <ros/ros.h>
#include <ros_android/main_thread.h>


namespace ros_android {
    std::mutex MainThread::s_instance_mutex;
    MainThread::Ptr MainThread::s_instance;
    MainThread::MainThread() : node_name("ros_android_node") {}
    MainThread::MainThread(std::string name) : node_name(name) {}
    MainThread::~MainThread() {}

    bool MainThread::check_ros_master(std::string master_ip, std::string my_ip) {
        int argc = 3;
        bool rv;

        const char* argv[] = {"nothing_important",
                                (std::string("__master:=http://") + master_ip + std::string(":11311")).c_str(),
                                (std::string("__ip:=") + my_ip).c_str()
                                };

        ros::init(argc, const_cast<char**>(&argv[0]), node_name.c_str());
        rv = ros::master::check();
        if (rv) {
            ROS_INFO("ROS MASTER IS UP!");
        } else {
            ROS_INFO("NO ROS MASTER.");
        }
        ros::start();
        return rv;
    }
}