#!/usr/bin/env python3

import rospy
import tkinter as tki
import threading
import signal
from geometry_msgs.msg import PoseStamped, Point, Pose
from std_msgs.msg import Empty, String
from tf.transformations import euler_from_quaternion
import math

class MultiTelloUI:
    def __init__(self, root):
        rospy.init_node('multi_tello_ui', anonymous=True)
        
        # Handle shutdown signals
        signal.signal(signal.SIGINT, self.onClose)
        signal.signal(signal.SIGTERM, self.onClose)
        
        self.quit_flag = False
        self.root = root
        self.drones = {}  # Dictionary to store drone data
        
        # Create main container frames
        self.control_frame = tki.Frame(root, relief=tki.SUNKEN)
        self.control_frame.grid(row=0, column=0, padx=5, pady=5)
        
        self.drone_frames = tki.Frame(root, relief=tki.SUNKEN)
        self.drone_frames.grid(row=1, column=0, padx=5, pady=5)
        
        # Add drone button
        self.add_drone_button = tki.Button(self.control_frame, text="Add Drone", command=self.add_drone_dialog)
        self.add_drone_button.grid(row=0, column=0, padx=5, pady=5)
        
        # Set window title
        self.root.wm_title("Multi-Tello Controller")
        self.root.wm_protocol("WM_DELETE_WINDOW", self.onClose)

    def add_drone_dialog(self):
        dialog = tki.Toplevel()
        dialog.title("Add New Drone")
        
        tki.Label(dialog, text="Enter Drone ID:").grid(row=0, column=0, padx=5, pady=5)
        id_entry = tki.Entry(dialog)
        id_entry.grid(row=0, column=1, padx=5, pady=5)
        
        def submit():
            drone_id = id_entry.get()
            if drone_id and drone_id not in self.drones:
                self.add_drone(drone_id)
                dialog.destroy()
        
        tki.Button(dialog, text="Add", command=submit).grid(row=1, column=0, columnspan=2, pady=10)

    def add_drone(self, drone_id):
        # Create frame for new drone
        drone_frame = tki.LabelFrame(self.drone_frames, text=f"Drone {drone_id}", relief=tki.SUNKEN, borderwidth=1)
        drone_frame.grid(row=len(self.drones), column=0, padx=5, pady=5, sticky='ew')
        
        # Initialize drone data structure
        self.drones[drone_id] = {
            'frame': drone_frame,
            'widgets': {},
            'data': {
                'altitude': 0.0,
                'battery': 0.0,
                'slam_pos': Point()
            }
        }
        
        # Create widgets for this drone
        self.create_drone_widgets(drone_id)
        
        # Subscribe to topics for this drone
        self.subscribe_to_drone_topics(drone_id)

    def create_drone_widgets(self, drone_id):
        drone = self.drones[drone_id]
        frame = drone['frame']
        widgets = drone['widgets']
        
        # Control buttons
        btn_frame = tki.Frame(frame)
        btn_frame.grid(row=0, column=0, columnspan=2, pady=5)
        
        widgets['takeoff'] = tki.Button(btn_frame, text="Takeoff", 
                                      command=lambda: self.takeoff(drone_id),
                                      bg='yellow')
        widgets['takeoff'].grid(row=0, column=0, padx=3)
        
        widgets['land'] = tki.Button(btn_frame, text="Land", 
                                   command=lambda: self.land(drone_id),
                                   bg='white')
        widgets['land'].grid(row=0, column=1, padx=3)
        
        widgets['emergency'] = tki.Button(btn_frame, text="EMERGENCY STOP", 
                                        command=lambda: self.emergency_stop(drone_id),
                                        bg='red', fg='white')
        widgets['emergency'].grid(row=0, column=2, padx=3)
        
        # Status displays
        status_frame = tki.Frame(frame)
        status_frame.grid(row=1, column=0, columnspan=2, pady=5)
        
        # Battery
        tki.Label(status_frame, text="Battery:").grid(row=0, column=0)
        widgets['battery'] = tki.Label(status_frame, text="0%")
        widgets['battery'].grid(row=0, column=1, padx=5)
        
        # Altitude
        tki.Label(status_frame, text="Altitude:").grid(row=0, column=2)
        widgets['altitude'] = tki.Label(status_frame, text="0.0m")
        widgets['altitude'].grid(row=0, column=3, padx=5)
        
        # SLAM Position
        slam_frame = tki.Frame(frame)
        slam_frame.grid(row=2, column=0, columnspan=2, pady=5)
        
        tki.Label(slam_frame, text="SLAM Position:").grid(row=0, column=0)
        widgets['slam_pos'] = tki.Label(slam_frame, text="x: 0.0, y: 0.0, z: 0.0")
        widgets['slam_pos'].grid(row=0, column=1, padx=5)

    def subscribe_to_drone_topics(self, drone_id):
        prefix = f"tello{drone_id}/"
        
        # Flight data (battery and altitude)
        rospy.Subscriber(prefix + 'flight_data', String, 
                        lambda msg: self.flight_data_callback(msg, drone_id))
        
        # SLAM position
        rospy.Subscriber("/exp"+ str(drone_id) + '/AirSLAM/frame_pose', PoseStamped, 
                        lambda msg: self.slam_callback(msg, drone_id))
        
        # Create publishers
        self.drones[drone_id]['publishers'] = {
            'takeoff': rospy.Publisher(prefix + 'takeoff', Empty, queue_size=1),
            'land': rospy.Publisher(prefix + 'land', Empty, queue_size=1),
            'emergency': rospy.Publisher(prefix + 'emergency', Empty, queue_size=1)
        }

    def flight_data_callback(self, flight_data, drone_id):
        if drone_id not in self.drones:
            return
            
        data_str = flight_data.data
        try:
            # Parse battery
            battery_start = data_str.find("Battery: ") + 9
            battery_end = data_str.find("%")
            battery = float(data_str[battery_start:battery_end])
            
            # Parse height
            height_start = data_str.find("Height: ") + 8
            height_end = data_str.find("m")
            altitude = float(data_str[height_start:height_end])
            
            # Update widgets
            self.drones[drone_id]['widgets']['battery'].config(text=f"{battery:.1f}%")
            self.drones[drone_id]['widgets']['altitude'].config(text=f"{altitude:.2f}m")
            
            # Store data
            self.drones[drone_id]['data']['battery'] = battery
            self.drones[drone_id]['data']['altitude'] = altitude
            
        except (ValueError, AttributeError) as e:
            rospy.logwarn(f"Error parsing flight data for drone {drone_id}: {e}")

    def slam_callback(self, slam_msg, drone_id):
        if drone_id not in self.drones:
            return
            
        pos = slam_msg.pose.position
        self.drones[drone_id]['data']['slam_pos'] = pos
        self.drones[drone_id]['widgets']['slam_pos'].config(
            text=f"x: {pos.x:.2f}, y: {pos.y:.2f}, z: {pos.z:.2f}")

    def takeoff(self, drone_id):
        if drone_id in self.drones:
            self.drones[drone_id]['publishers']['takeoff'].publish(Empty())
            self.drones[drone_id]['widgets']['takeoff'].configure(fg='green', bg='white')
            self.drones[drone_id]['widgets']['land'].configure(fg='black', bg='yellow')

    def land(self, drone_id):
        if drone_id in self.drones:
            self.drones[drone_id]['publishers']['land'].publish(Empty())
            self.drones[drone_id]['widgets']['takeoff'].configure(fg='black', bg='yellow')
            self.drones[drone_id]['widgets']['land'].configure(fg='green', bg='white')

    def emergency_stop(self, drone_id):
        if drone_id in self.drones:
            self.drones[drone_id]['publishers']['emergency'].publish(Empty())

    def onClose(self, *args):
        print("[INFO] closing...")
        self.quit_flag = True
        for drone_id in list(self.drones.keys()):
            try:
                self.land(drone_id)
            except:
                pass
        self.root.quit()
        self.root.destroy()

if __name__ == '__main__':
    root = tki.Tk()
    app = MultiTelloUI(root)
    root.mainloop()