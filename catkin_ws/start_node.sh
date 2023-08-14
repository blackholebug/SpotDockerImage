cd ./catkin_ws
catkin_make
source devel/setup.bash

python3.8 -m pip install -r src/spot_fsm_control/requirements.txt

roslaunch spot_fsm_control spot_fsm_control.launch