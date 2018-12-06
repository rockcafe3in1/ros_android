FROM ros:kinetic-ros-base-xenial
MAINTAINER Julian Cerruti <jcerruti@ekumenlabs.com>
MAINTAINER Gary Servin <gary@ekumenlabs.com>
MAINTAINER Juan Ignacio Ubeira <jubeira@ekumenlabs.com>

# Install basic tools
RUN apt-get update && apt-get install -y wget git unzip tmux python-catkin-tools python-lxml automake autoconf

# Install Android NDK
WORKDIR /opt/android/sdk
RUN wget https://dl.google.com/android/repository/android-ndk-r18b-linux-x86_64.zip
RUN unzip android-ndk-r18b-linux-x86_64.zip -d /opt/android/sdk

# Set-up environment
ENV ANDROID_NDK /opt/android/sdk/android-ndk-r18b

# Install g++ to avoid "CMAKE_CXX_COMPILER-NOTFOUND was not found." error
RUN apt-get update && apt-get install -y g++ cmake make clang

# Install Android SDK
ENV ANDROID_HOME /opt/android/sdk
RUN apt-get update && apt-get install -y openjdk-8-jdk
RUN wget https://dl.google.com/android/repository/sdk-tools-linux-4333796.zip
RUN unzip sdk-tools-linux-4333796.zip -d /opt/android/sdk
RUN yes | $ANDROID_HOME/tools/bin/sdkmanager --licenses
RUN $ANDROID_HOME/tools/bin/sdkmanager "build-tools;26.0.2" "platform-tools" "platforms;android-26" --sdk_root=/opt/android/sdk
ENV PATH /opt/android/sdk/tools:/opt/android/sdk/platform-tools:$PATH

# Install CMake 3.6
WORKDIR /opt
RUN wget https://cmake.org/files/v3.6/cmake-3.6.3-Linux-x86_64.sh && chmod +x cmake-3.6.3-Linux-x86_64.sh
RUN yes | sh '/opt/cmake-3.6.3-Linux-x86_64.sh'
RUN ln -s /opt/cmake-3.6.3-Linux-x86_64/bin/* -t /usr/local/bin

# Install Python libraries
RUN apt-get install python-lxml -y

ENV JAVA_HOME /usr/lib/jvm/java-8-openjdk-amd64/

# Use ccache
RUN apt-get update && apt-get install -y ccache
ENV USE_CCACHE 1
ENV NDK_CCACHE /usr/bin/ccache
ENV CCACHE_DIR /opt/roscpp_output/ccache
ENV PATH /usr/lib/ccache:$PATH
