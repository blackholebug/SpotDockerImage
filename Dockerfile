# syntax=docker/dockerfile:1
# FROM ubuntu:20.04
FROM osrf/ros:noetic-desktop
EXPOSE 10000
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

COPY . .


RUN apt-get update && apt-get install -y \
    python3-pip

RUN source ./ros_entrypoint.sh
# RUN source /opt/ros/noetic/setup.bash
# RUN rosdep update

RUN cd ./catkin_ws
# RUN cd ./catkin_ws/src/spot_fsm_control
# RUN echo $PWD
# RUN python3.8 -m pip install -r requirements.txt


# RUN cd ../..
# RUN catkin_make
# RUN source devel/setup.bash




# RUN rosrun spot_fsm_control fsm_node.py


