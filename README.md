# Tello_ROS
## Download Tello_ROS
```
cd ~
mkdir Tello_ros
cd Tello_ros
mkdir src
cd src
git clone https://github.com/vvEverett/tello_ros.git
```
## Download DJITelloPy
Recommand
```
cd ~/Tello_ros/src/tello_ros/src
git clone https://github.com/vvEverett/DJITelloPy.git
cd DJITelloPy
pip install -e .
```
or
```
pip3 install djitellopy
```
## Install
```
cd ~/Tello_ros
catkin_make
source ~/Tello_ros/devel/setup.bash
```
## Examples
Take Picture
```
roslaunch tello_ros take_pictures.launch
```
Then you can run
```
rosservice call /tello/take_picture
```
to take

Keyboard Control Tello
```
roslaunch tello_ros keyboard_control.launch
```
