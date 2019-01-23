#include <functional>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <ros_android/main_thread.h>
#include <std_msgs/String.h>

class HelloRos : public ros_android::MainThread {
  public:
    HelloRos() : ros_android::MainThread("hello_ros") {}

    virtual void run() override {
      ros::NodeHandle n;

      /* Write your main code here */
      ROS_INFO("GOING TO PUBLISHER");

      // Creating a publisher and a subscriber
      // When something is received in chatter topic, a message is published in a_chatter topic
      chatter_pub = n.advertise<std_msgs::String>("a_chatter", 1000);
      ros::Subscriber sub = n.subscribe<std_msgs::String>("chatter", 1000, std::bind(&HelloRos::chatterCallback, this, std::placeholders::_1));

      // Rate 1Hz
      ros::WallRate loop_rate(1);

      while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
      }
    }

    virtual void stop() override {
      /* Write your clean-up code here */
      ros::shutdown();
    }

  private:
    int loop_count_ = 0;
    ros::Publisher chatter_pub;

    void chatterCallback(const std_msgs::String::ConstPtr& msg){
      ROS_INFO("%s", msg->data.c_str());
      loop_count_++;
      std_msgs::String msgo;
      std::stringstream ss;
      ss << "hello world from android ndk " << loop_count_;
      msgo.data = ss.str();
      chatter_pub.publish(msgo);
      ROS_INFO("%s", msg->data.c_str());
    }
};

MY_ROS_ANDROID_MAIN_THREAD(HelloRos)
