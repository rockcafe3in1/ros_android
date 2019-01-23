#include <ros_android/main_thread.h>
#include "com_ros_example_hello_ros_MainActivity_RosThread.h"

using namespace ros_android;

JNIEXPORT void JNICALL Java_com_ros_example_hello_1ros_MainActivity_00024RosThread_run
  (JNIEnv *, jobject) {
  MainThread::Instance()->run();
}

JNIEXPORT void JNICALL Java_com_ros_example_hello_1ros_MainActivity_00024RosThread_stop
  (JNIEnv *, jobject) {
  MainThread::Instance()->stop();
}

JNIEXPORT jboolean JNICALL Java_com_ros_example_hello_1ros_MainActivity_00024RosThread_checkRosMaster
  (JNIEnv *env, jobject, jstring j_master_ip, jstring j_my_ip) {
  return MainThread::Instance()->check_ros_master(env->GetStringUTFChars(j_master_ip, nullptr), env->GetStringUTFChars(j_my_ip, nullptr));
}

JNIEXPORT void JNICALL Java_com_ros_example_hello_1ros_MainActivity_00024RosThread__1_1RosThread
  (JNIEnv *, jobject) {
  MainThread::Instance();
}