cd ./catkin_ws
catkin_make
source devel/setup.bash

dos2unix /catkin_ws/src/ros_tcp_endpoint/src/ros_tcp_endpoint/default_server_endpoint.py

python3.8 -m pip install -r src/spot_fsm_control/requirements.txt

roslaunch spot_fsm_control spot_fsm_control.launch