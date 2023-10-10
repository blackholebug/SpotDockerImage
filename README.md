# SpotDockerImage
Catkin workspace to control Spot from Boston Dynamics.
HTC Vive trackers are used to determine the 6D position of Spot.

Can be either run in Docker or on Windows with a ROS installation.
Docker does not allow for communication with the HTC Vive Tracker.

## Idea
The ROS nodes connect to Spot and are able to control the robot, based on ros message being send to the nodes. 
Allows to connect any interface (for example HoloLens) and control the robot's movement and arm manipulation.
Actions are controlled by a state machine to allow for serialized execution of actions.

## Versions
* ROS Noetic
* Python 3.8.10
* bosdyn 3.2.0 PyPi packages

## Run the image
Run the nodes on docker. Make sure to open docker desktop in Windows.
```bash
docker build -t spot_docker_image .
# Open the port connection to connect Unity ROS-TCP-Connector with the Docker image
docker run -it -p 10000:10000 spot_docker_image

```

## Run on Windows
Open a terminal that has ROS source and navigate to the ```catkin_ws``` folder

For the first time run the make script to compile the ROS nodes and install the python packages
```bash
make.bat
```

If in the same terminal, you can now run only launching the ROS nodes without sourcing.
```bash
start.bat
```
