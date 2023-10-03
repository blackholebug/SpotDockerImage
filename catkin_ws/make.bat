catkin_make
cd devel
call setup.bat 
cd ..
python -m pip install --upgrade pip
python -m pip install -r requirements.txt

roslaunch spot_fsm_control spot_fsm_control.launch