#FROM osrf/ros:noetic-desktop-full
FROM nag92/drexel_lab_core:latest
#FROM my_build
#FROM drexel_labs
# # Minimal setup

RUN dpkg-reconfigure locales \
&& apt-get update \
&& apt-get install -y locales lsb-release \
&& apt install ros-noetic-joint-trajectory-controller \
&& sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
&& apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
&& apt-get update \
&& apt-get install -y --no-install-recommends python3-rosdep gedit


# Install turtlebot stuff

# install git
RUN apt-get update && apt-get install -y git


# # Build the Catkin workspace and ensure it's sourced
RUN source /opt/ros/noetic/setup.bash \
 && cd /turtlebot3_ws \
 && catkin_make \
 && source /turtlebot3_ws/devel/setup.bash
RUN echo "source /turtlebot3_ws/devel/setup.bash" >> ~/.bashrc

RUN echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc
RUN echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc

RUN echo "export OPENCR_PORT=/dev/ttyACM0" >> ~/.bashrc
RUN echo "export OPENCR_MODEL=om_with_tb3_noetic" >> ~/.bashrc
RUN  echo "export LDS_MODEL=LDS-02" >> ~/.bashrc
# Set the working folder at startup
# RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

WORKDIR  C:\Users\jeff\Desktop\Masters Organized\Intro to Robotics, MEM 571\Labs\robotics vscode folder\labs-WeffreyJ\catkin_ws

# Change the default shell to Bash

SHELL [ "/bin/bash" , "-c" ]


# Set the DISPLAY environment variable for X11
ENV DISPLAY host.docker.internal:0.0

# Set other environment variables if needed
ENV DISPLAY: unix :0
 ENV ROS_LOCALHOST_ONLY 1
 ENV ROS_DOMAIN_ID 42

# ... continue with the rest of your Dockerfile ...
