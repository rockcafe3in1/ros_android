This is a hello ROS example app. It subscribes to `/chatter` topic and when a message is received, the following message is published in `/a_chatter`:

    data: "hello world from android ndk __COUNTER_VALUE__"

USAGE
-------

1. IP addresses are hardcoded, so you must edit the master URI and the android device ip address in the following file:

        app/src/main/cpp/main.cpp

2. Build the samples with do_everything script.

3. Install the app in your android device using adb -d install as explained here:
        <https://developer.android.com/studio/build/building-cmdline>

4. Execute roscore in a terminal with ros sourced. Remember to export first your ip address:
        
        export ROS_IP=__YOUR_IP_ADDRESS__

5. Subscribe to /a_chatter in another terminal with ros setup sourced:
    
        rostopic echo /a_chatter

6. Publish to /chatter, in another terminal with ros setup sourced and ROS_IP exported:

        rostopic pub /chatter std_msgs/String "__YOUR_MESSAGE__" -r __PUBLISHING_RATE__

If all is working well, you will receive multiple messages with an incresing counter value.
The message you have sent is logged in android, you can check the result with logcat:

    adb logcat

The log will be like:

    12-13 15:53:36.449 12078 12093 I ROSCPP_NDK_EXAMPLE: __YOUR_MESSAGE__
