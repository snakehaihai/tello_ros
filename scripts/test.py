
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Point, Pose, Quaternion, PoseArray
from std_msgs.msg import Empty, Bool, Int32, Float32
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
import threading
import math
import numpy as np
def deg_to_rad(deg):
    return deg * math.pi / 180.0

Kp = Point(0.6, 0.6, 1.5)
Kd = Point(1.0, 1.0, 1.5)
Kp_yaw = 0.015 # 0.005
Kd_yaw = 0.01 # 0.2 = 360 deg / 38 sec = 9 deg/sec
command_pos = Point(1.0, 4.0, 1.5)
real_world_pos = Point(3.0, 1.0, 1.5)
pos_error = Point()
pos_error.x = command_pos.x - real_world_pos.x
pos_error.y = command_pos.y - real_world_pos.y
pos_error.z = command_pos.z - real_world_pos.z
command_orientation_deg = 0.0
slam_orientation_deg =30.0
max_val = 100000000
if abs(command_orientation_deg - slam_orientation_deg) < max_val:
    error = command_orientation_deg - slam_orientation_deg
    max_val = abs(error)
if abs(command_orientation_deg - slam_orientation_deg + 360) < max_val:
    error = command_orientation_deg - slam_orientation_deg + 360
    max_val = abs(error)
if abs(command_orientation_deg - slam_orientation_deg - 360) < max_val:
    error = command_orientation_deg - slam_orientation_deg - 360
    max_val = abs(error)

rotation_error_deg = error


slam_orientation_rad = deg_to_rad(slam_orientation_deg)
x_rotated = pos_error.x*math.cos(slam_orientation_rad) + pos_error.y*math.sin(slam_orientation_rad)
y_rotated = pos_error.y*math.cos(slam_orientation_rad) - pos_error.x*math.sin(slam_orientation_rad)
x_change = y_rotated
y_change = x_rotated
pos_error.x = x_change
pos_error.y = y_change
print("pos_error: ", pos_error)

pos_err_derivative = Point()
pos_error_prev = Point()
rotation_error_prev=0.0
rotation_err_filtered_derivative = 0.0

pos_err_derivative.x = pos_error.x - pos_error_prev.x
pos_err_derivative.y = pos_error.y - pos_error_prev.y
pos_err_derivative.z = pos_error.z - pos_error_prev.z
rotation_err_derivative = rotation_error_deg - rotation_error_prev

pos_err_filtered_derivative = Point()

pos_err_filtered_derivative.x = 0.5648*pos_err_filtered_derivative.x + 12.75*pos_err_derivative.x
pos_err_filtered_derivative.y = 0.5648*pos_err_filtered_derivative.y + 12.75*pos_err_derivative.y
pos_err_filtered_derivative.z = 0.5648*pos_err_filtered_derivative.z + 12.75*pos_err_derivative.z
rotation_err_filtered_derivative = 0.5648*rotation_err_filtered_derivative + 12.75*rotation_err_derivative

speed = Point()
speed.x = (Kp.x*pos_error.x + Kd.x*pos_err_filtered_derivative.x)/16
speed.y = (Kp.y*pos_error.y + Kd.y*pos_err_filtered_derivative.y)/16
speed.z = (Kp.z*pos_error.z + Kd.z*pos_err_filtered_derivative.z)/16
yaw = (Kp_yaw*rotation_error_deg + Kd_yaw*rotation_err_filtered_derivative)/16
yaw = -yaw


print("speed: ", speed)
print("yaw: ", yaw)