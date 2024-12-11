# Tello_ROS
# 💻Download and Install
## 🏁 Test Environment
**Dependencies**
- Ubuntu 22.04
- ROS Noetic
- Python 3.10.12

## Download Tello_ROS
Download our respositoty.
```
cd ~
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
git clone https://github.com/vvEverett/tello_ros.git
```
## Download DJITelloPy
Install our version of DJITelloPy. We modified some functions to grab multi tello video stream.
```
cd ~/catkin_ws/src/tello_ros/src
git clone https://github.com/vvEverett/DJITelloPy.git
cd DJITelloPy
pip install -e .
```
## AirSLAM (Optional)
You need to download and install AirSLAM or any other SLAM algorithm in same ROS working space if you want to use SLAM to locate Tello.

For detailed installation guide, visit [AirSLAM Repository](https://github.com/vvEverett/AirSLAM) (We have made modifications to adapt for **tello_ros**).
```
cd ~/catkin_ws/src
git clone https://github.com/vvEverett/AirSLAM.git
```
## Install
```
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```
# 🏃Usage
## Take Pictures

```
roslaunch tello_ros take_pictures.launch
```
And run following command to take picture.
```
rosservice call /tello/take_picture
```
## Keyboard Control Tello
```
roslaunch tello_ros keyboard_control.launch
```
## AirSLAM Tello UI
```
roslaunch tello_ros reloc_tello_slam.launch
```
## Connect to Multiple Tellos
```
roslaunch tello_ros multi-tello.launch
```
## AirSLAM Multiple Tellos UI
```
roslaunch tello_ros reloc_tello_slam_multi.launch
```
## AirSLAM Multiple Tellos Formation UI
```
roslaunch tello_ros NTU.launch
```