cd ./catkin_ws
catkin_make
source devel/setup.bash

python3.8 -m pip install -r src/spot_fsm_control/requirements.txt

# rosrun spot_fsm_control fsm_node.py