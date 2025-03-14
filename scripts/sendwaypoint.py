#!/usr/bin/env python3

# This script sends waypoint commands to the Tello drones
# The waypoints are defined in the WAYPOINTS dictionary
import rospy
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import math

# Define waypoints for each drone
# Format: waypoint_name: {drone_id: [x, y, z, yaw]}

WAYPOINTS = {
    'point1': {
        '0': [-0.60, 0.20, 1.8, -18.0],
        '1': [0.20, 1.20, 1.8, 0.0],
        '2': [1.1, 1.2, 1.8, 30.0]
    },
    'point2': {
        '0': [-0.60, 1.20, 1.8, -18.0],
        '1': [1.0, 1.20, 1.8, 0.0],
        '2': [1.1, 0.2, 1.8, 30.0]
    },
    'point3': {
        '0': [0.10, 0.20, 1.8, -18.0],
        '1': [0.6, 1.20, 1.8, 0.0],
        '2': [1.8, 0.2, 1.8, 30.0]
    },
    'point4': {
        '0': [0.10, 1.20, 1.8, -18.0],
        '1': [0.6, 0.20, 1.8, 0.0],
        '2': [1.8, 1.2, 1.8, 30.0]
    }
}

class WaypointSender:
    def __init__(self):
        # initialize ROS node
        rospy.init_node('waypoint_publisher', anonymous=True)
        
        # create publishers for each drone
        self.publishers = {}
        for drone_id in WAYPOINTS['point1'].keys():  # Use the first waypoint to get the drone IDs
            topic = f"tello{drone_id}/command_pos"
            self.publishers[drone_id] = rospy.Publisher(topic, Pose, queue_size=1)
        
        # wait for publishers to connect
        rospy.sleep(0.5)

    def send_waypoint(self, waypoint_name):
        if waypoint_name not in WAYPOINTS:
            print(f"Error: Cannot find waypoint '{waypoint_name}'")
            return
            
        waypoint = WAYPOINTS[waypoint_name]
        print(f"\nSending {waypoint_name} position command...")
        
        # Send the waypoint command to each drone
        for drone_id, pos in waypoint.items():
            x, y, z, yaw_deg = pos
            
            command = Pose()
            command.position.x = x
            command.position.y = y
            command.position.z = z
            
            # Convert yaw angle from degrees to radians
            yaw_rad = math.radians(yaw_deg)
            q = quaternion_from_euler(0, 0, yaw_rad)
            command.orientation.x = q[0]
            command.orientation.y = q[1]
            command.orientation.z = q[2]
            command.orientation.w = q[3]
            
            # Publish the command multiple times to ensure it is received
            for _ in range(3):
                self.publishers[drone_id].publish(command)
                rospy.sleep(0.1)
            
            print(f"Drone {drone_id}: x={x}, y={y}, z={z}, yaw={yaw_deg}°")

    def run(self):
        while not rospy.is_shutdown():
            print("\nAvailable waypoints:")
            for point in WAYPOINTS.keys():
                print(f"- {point}")
            print("\nplease input waypoint name:")
            
            cmd = input().strip()
            if cmd.lower() == 'q':
                break
                
            if cmd in WAYPOINTS:
                self.send_waypoint(cmd)
            else:
                print(f"Error: invalid waypoint '{cmd}'")

if __name__ == "__main__":
    try:
        sender = WaypointSender()
        sender.run()
    except Exception as e:
        print(f"Error: {str(e)}")