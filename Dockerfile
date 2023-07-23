FROM ubuntu:20.04

RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

RUN sudo apt install curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

RUN sudo apt update
RUN sudo apt install python3.8 python3.8-pip
RUN sudo apt install ros-noetic-ros-base

RUN source /opt/ros/noetic/setup.bash
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN source ~/.bashrc

RUN sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
RUN sudo apt install python3-rosdep

RUN sudo rosdep init
RUN rosdep update

COPY /catkin_ws/ ~/

RUN cd ~/catkin_ws
RUN catkin_make
RUN source devel/setup.bash

RUN cd src/spot_fsm_control
RUN python3.8 -m pip install -r requirements.txt
RUN rosrun spot_fsm_control fsm_node.py
