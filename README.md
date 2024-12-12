# Tello_ROS
A ROS node for controlling DJI Tello drones with comprehensive control, monitoring, and multi-drone formation capabilities.
# 💻Download and Install
## Test Environment
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
## Install modified DJITelloPy (with multi-drone video stream support)
Install our version of DJITelloPy. We modified some functions to grab multi tello video stream.
```
cd ~/catkin_ws/src/tello_ros/src
git clone https://github.com/vvEverett/DJITelloPy.git
cd DJITelloPy
pip install -e .
```
## Install AirSLAM (Optional, for drone localization)
You need to download and install AirSLAM or any other SLAM algorithm in same ROS working space if you want to use SLAM to locate Tello.

For detailed installation guide, visit [AirSLAM Repository](https://github.com/vvEverett/AirSLAM) (We have made modifications to adapt for **tello_ros**).
```
cd ~/catkin_ws/src
git clone https://github.com/vvEverett/AirSLAM.git
```
## Build and Source
```
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```
# 🚁 Features
## Node Parameters
### Single Drone Configuration

| Parameter | Default | Description |
|-----------|---------| ------------|
| ~ID | '' | Drone identifier |
| ~drone_ip | '192.168.10.1' | Drone IP address |
| ~video_port | '11111' | Video stream port |
### Multi-Drone Configuration
Configuration in launch file:
```yaml
tello_configs:
  - id: "0"
    ip: "192.168.10.1"
    video_port: 11111
  - id: "1"
    ip: "192.168.3.21"
    video_port: 11118
  - id: "2"
    ip: "192.168.3.22"
    video_port: 11119
```
## Published Topics

For each drone (replace {ID} with drone identifier):

| Topic | Message Type | Rate | Description |
|-------|--------------|------|-------------|
| tello{ID}/camera/image_raw | sensor_msgs/Image | 30Hz | RGB camera image |
| tello{ID}/camera/image_gray | sensor_msgs/Image | 30Hz | Grayscale camera image |
| tello{ID}/altitude | std_msgs/Float32 | 10Hz | Height data (meters) |
| tello{ID}/battery | std_msgs/Int32 | 10Hz | Battery percentage |
| tello{ID}/flight_data | std_msgs/String | 10Hz | Comprehensive flight status |
| tello{ID}/flight/attitude | geometry_msgs/Vector3Stamped | 10Hz | Attitude data (degrees) |
| tello{ID}/flight/velocity | geometry_msgs/TwistStamped | 10Hz | Velocity data (m/s) |

## Subscribed Topics

For each drone (replace {ID} with drone identifier):

| Topic | Message Type | Function |
|-------|--------------|----------|
| tello{ID}/cmd_vel | geometry_msgs/Twist | Velocity control (-1 to 1 range) |
| tello{ID}/takeoff | std_msgs/Empty | Takeoff command |
| tello{ID}/land | std_msgs/Empty | Landing command |
| tello{ID}/emergency | std_msgs/Empty | Emergency stop |
# 🎮Usage
## Basic Functions
### Take Pictures
```
# Launch camera node
roslaunch tello_ros take_pictures.launch

# Trigger picture capture
rosservice call /tello/take_picture
```
### Flight Control Examples
```
# Takeoff and Land control
rostopic pub /tello/takeoff std_msgs/Empty "{}"
rostopic pub /tello/land std_msgs/Empty "{}"

# Velocity control
rostopic pub /tello/cmd_vel geometry_msgs/Twist "linear:
  x: 0.5  # Forward/backward (-1.0 to 1.0)
  y: 0.0  # Left/right (-1.0 to 1.0)
  z: 0.0  # Up/down (-1.0 to 1.0)
angular:
  z: 0.0" # Yaw rotation (-1.0 to 1.0)
# Emergency Stop
rostopic pub /tello/emergency std_msgs/Empty "{}"
```
### Keyboard Control
```
roslaunch tello_ros keyboard_control.launch
```
### Single Drone SLAM UI (With AirSLAM)
```
roslaunch tello_ros reloc_tello_slam.launch
```
## Multi-Drone Operations
### Launch Multiple Drones
```
roslaunch tello_ros multi-tello.launch
```
### Multi-Drone SLAM UI (With AirSLAM)
```
roslaunch tello_ros reloc_tello_slam_multi.launch
```
### Formation Control UI (With AirSLAM)
```
roslaunch tello_ros NTU.launch
```

# ⚠️ Safety Features

- Independent video stream handler with auto-retry mechanism
- Thread-safe frame capture and processing
- Emergency stop via topic
- Automatic landing on node shutdown
- Exception handling for connection issues
- Continuous status monitoring and logging
- Separate cleanup routines for each drone

# 📋 Important Notes

1. Ensure sufficient battery charge before flight
2. Verify WiFi connection before operation
3. For multi-drone setup:
   - Configure unique IP addresses for each drone
   - Ensure video ports don't conflict
   - Monitor network bandwidth usage
4. Test commands in simulator first
5. Keep safe distance between drones in formation flight

# 🔧 Troubleshooting

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| Connection Failed | WiFi Not Connected | Check WiFi Connection |
| Video Stream Interruption | Network Instability | Reduce Distance to Drone |
| Multi-drone Control Issues | IP/Port Conflicts | Verify Unique Configurations |
| Unresponsive Commands | Low Battery | Check Battery Level |
| Video Quality Issues | Bandwidth Limitations | Adjust Video Resolution/FPS |

# 🔗 Related Links

- [AirSLAM Repository](https://github.com/sair-lab/AirSLAM)
- [DJITelloPy Repository](https://github.com/damiafuentes/DJITelloPy)
# 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.