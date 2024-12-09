#!/usr/bin/env python3

import rospy
import tkinter as tki
from tkinter import ttk
import threading
import signal
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Empty, String, Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

class MultiTelloUI:
    def __init__(self, root):
        rospy.init_node('multi_tello_ui', anonymous=True)
        
        signal.signal(signal.SIGINT, self.onClose)
        signal.signal(signal.SIGTERM, self.onClose)
        
        self.quit_flag = False
        self.root = root
        self.drones = {}  # Dictionary to store drone data
        
        # Base scale for trajectories (can be adjusted by scale factor)
        self.base_scale = 1.0
        # Define relative trajectories (will be adjusted based on current position)
        self.trajectories = {
            'N': [
                [0.0, 0.0, 0.0, 0],     # Start (will be replaced with current position)
                [0.0, 0.0, 0.3, 0],     # Up
                [0.5, 0.0, 0.0, 0],     # Forward
                [-0.5, 0.0, 0.0, 0],    # Back to middle
                [0.2, 0.0, 0.0, 0],    # Backward
                [-0.2, 0.0, -0.3, 0],    # Back to start height
            ],
            'T': [
                [0.0, 0.0, 0.0, 0],     # Start
                [0.0, 0.0, 0.3, 0],     # Up
                [-1.0, 0.0, 0.0, 0],    # Left
                [2.0, 0.0, 0.0, 0],     # Right
                [-1.0, 0.0, 0.0, 0],    # Center
                [0.0, -1.0, 0.0, 0],    # Down
                [0.0, 1.0, -0.3, 0],    # Back to start height
            ],
            'U': [
                [0.0, 0.0, 0.0, 0],     # Start
                [0.0, 0.0, 0.3, 0],     # Up
                [0.0, 1.0, 0.0, 0],     # Forward top
                [0.0, -2.0, 0.0, 0],    # Down
                [1.0, 0.0, 0.0, 0],     # Right
                [0.0, 2.0, 0.0, 0],     # Up right
                [-1.0, -1.0, -0.3, 0],  # Back to start
            ]
        }
        
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
        drone_frame = tki.LabelFrame(self.drone_frames, text=f"Drone {drone_id}", relief=tki.SUNKEN, borderwidth=1)
        drone_frame.grid(row=len(self.drones), column=0, padx=5, pady=5, sticky='ew')
        
        self.drones[drone_id] = {
            'frame': drone_frame,
            'widgets': {},
            'data': {
                'altitude': 0.0,
                'battery': 0.0,
                'slam_pos': Point(),
                'real_world_pos': Point(),
                'trajectory_active': False,
                'current_trajectory': None,
                'has_position_data': False,
                'allow_slam_control': False,
                'start_position': None  # Store starting position for relative trajectories
            },
            'trajectory_threshold': Point(0.3, 0.3, 0.3)
        }
        
        self.create_drone_widgets(drone_id)
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
        
        widgets['slam_control'] = tki.Button(btn_frame, text="Enable SLAM Control", 
                                           command=lambda: self.toggle_slam_control(drone_id),
                                           bg='yellow')
        widgets['slam_control'].grid(row=0, column=3, padx=3)
        
        # Trajectory control
        trajectory_frame = tki.Frame(frame)
        trajectory_frame.grid(row=1, column=0, columnspan=2, pady=5)
        
        tki.Label(trajectory_frame, text="Trajectory:").grid(row=0, column=0)
        widgets['trajectory'] = ttk.Combobox(trajectory_frame, 
                                           values=list(self.trajectories.keys()),
                                           width=10)
        widgets['trajectory'].grid(row=0, column=1, padx=5)
    
        widgets['start_trajectory'] = tki.Button(trajectory_frame, 
                                               text="Start Trajectory",
                                               command=lambda: self.start_trajectory(drone_id),
                                               bg='yellow')
        widgets['start_trajectory'].grid(row=0, column=2, padx=3)
        
        widgets['stop_trajectory'] = tki.Button(trajectory_frame,
                                              text="Stop Trajectory",
                                              command=lambda: self.stop_trajectory(drone_id),
                                              bg='white')
        widgets['stop_trajectory'].grid(row=0, column=3, padx=3)
        
        # Add scale slider
        tki.Label(trajectory_frame, text="Scale:").grid(row=0, column=4)
        widgets['scale'] = tki.Scale(trajectory_frame, 
                                    from_=0.3, 
                                    to=2.0,
                                    resolution=0.1,
                                    orient=tki.HORIZONTAL,
                                    length=100)
        widgets['scale'].set(1.0)
        widgets['scale'].grid(row=0, column=5, padx=5)
        
        # Status displays
        status_frame = tki.Frame(frame)
        status_frame.grid(row=2, column=0, columnspan=2, pady=5)
        
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
        slam_frame.grid(row=3, column=0, columnspan=2, pady=5)
        
        tki.Label(slam_frame, text="SLAM Position:").grid(row=0, column=0)
        widgets['slam_pos'] = tki.Label(slam_frame, text="x: 0.0, y: 0.0, z: 0.0")
        widgets['slam_pos'].grid(row=0, column=1, padx=5)
        
        # Add trajectory points display
        trajectory_points_frame = tki.Frame(frame)
        trajectory_points_frame.grid(row=4, column=0, columnspan=2, pady=5)
        
        tki.Label(trajectory_points_frame, text="Trajectory Points:").grid(row=0, column=0, sticky='w')
        
        # Create text widget to display points
        widgets['trajectory_points'] = tki.Text(trajectory_points_frame, height=6, width=50)
        widgets['trajectory_points'].grid(row=1, column=0, padx=5)
        widgets['trajectory_points'].config(state='disabled')
        
        # Add current point indicator
        tki.Label(trajectory_points_frame, text="Current Point:").grid(row=2, column=0, sticky='w')
        widgets['current_point'] = tki.Label(trajectory_points_frame, text="Not started")
        widgets['current_point'].grid(row=3, column=0, sticky='w', padx=5)

    def subscribe_to_drone_topics(self, drone_id):
        prefix = f"tello{drone_id}/"
        
        # Flight data (battery and altitude)
        rospy.Subscriber(prefix + 'flight_data', String, 
                        lambda msg: self.flight_data_callback(msg, drone_id))
        
        # SLAM position with updated topic
        rospy.Subscriber("/exp" + str(drone_id) + '/AirSLAM/frame_pose', PoseStamped, 
                        lambda msg: self.slam_callback(msg, drone_id))
        
        # Real world position
        rospy.Subscriber(prefix + 'real_world_pos', PoseStamped,
                        lambda msg: self.real_world_pos_callback(msg, drone_id))
        
        # Allow SLAM control status
        rospy.Subscriber(prefix + 'allow_slam_control', Bool,
                        lambda msg: self.slam_control_callback(msg, drone_id))
        
        # Create publishers
        self.drones[drone_id]['publishers'] = {
            'takeoff': rospy.Publisher(prefix + 'takeoff', Empty, queue_size=1),
            'land': rospy.Publisher(prefix + 'land', Empty, queue_size=1),
            'emergency': rospy.Publisher(prefix + 'emergency', Empty, queue_size=1),
            'command_pos': rospy.Publisher(prefix + 'command_pos', Pose, queue_size=1),
            'path': rospy.Publisher(prefix + 'path', Path, queue_size=1),
            'allow_slam_control': rospy.Publisher(prefix + 'allow_slam_control', Bool, queue_size=1)
        }

    def convert_to_absolute_trajectory(self, relative_trajectory, start_pos, scale):
        """Convert relative trajectory to absolute based on starting position and scale"""
        absolute_trajectory = []
        current_pos = [start_pos.x, start_pos.y, start_pos.z, 0]
        
        for point in relative_trajectory:
            # Scale x, y, z coordinates (but not yaw)
            absolute_point = [
                current_pos[0] + point[0] * scale,
                current_pos[1] + point[1] * scale,
                current_pos[2] + point[2] * scale,
                point[3]  # Keep original yaw
            ]
            absolute_trajectory.append(absolute_point)
            # Update current position with scaled coordinates
            current_pos = absolute_point.copy()
            
        return absolute_trajectory

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
        self.drones[drone_id]['data']['has_position_data'] = True

    def real_world_pos_callback(self, msg, drone_id):
        if drone_id not in self.drones:
            return
        self.drones[drone_id]['data']['real_world_pos'] = msg.pose.position
        self.drones[drone_id]['data']['has_position_data'] = True

    def slam_control_callback(self, msg, drone_id):
        if drone_id not in self.drones:
            return
            
        drone = self.drones[drone_id]
        drone['data']['allow_slam_control'] = msg.data
        
        if msg.data:
            drone['widgets']['slam_control'].configure(text="Disable SLAM Control", 
                                                     bg='black', fg='yellow')
        else:
            drone['widgets']['slam_control'].configure(text="Enable SLAM Control", 
                                                     bg='yellow', fg='black')

    def toggle_slam_control(self, drone_id):
        if drone_id not in self.drones:
            return
            
        drone = self.drones[drone_id]
        current_state = drone['data']['allow_slam_control']
        new_state = not current_state
        
        drone['data']['allow_slam_control'] = new_state
        drone['publishers']['allow_slam_control'].publish(Bool(new_state))
        
        if new_state:
            drone['widgets']['slam_control'].configure(text="Disable SLAM Control", 
                                                     bg='black', fg='yellow')
        else:
            drone['widgets']['slam_control'].configure(text="Enable SLAM Control", 
                                                     bg='yellow', fg='black')
        rospy.loginfo(f"SLAM Control for drone {drone_id} {'enabled' if new_state else 'disabled'}")

    def start_trajectory(self, drone_id):
        if drone_id not in self.drones:
            return
            
        drone = self.drones[drone_id]
        
        if not drone['data']['allow_slam_control']:
            rospy.logwarn(f"Cannot start trajectory - SLAM Control not enabled for drone {drone_id}")
            return
            
        selected_trajectory = drone['widgets']['trajectory'].get()
        
        if selected_trajectory not in self.trajectories:
            rospy.logwarn(f"No trajectory selected for drone {drone_id}")
            return
            
        if not drone['data']['has_position_data']:
            rospy.logwarn(f"Waiting for position data for drone {drone_id}")
            return

        if not drone['data']['trajectory_active']:
            # Get current scale value
            scale = float(drone['widgets']['scale'].get())
            
            # Store current position as start position
            start_pos = drone['data']['real_world_pos']
            drone['data']['start_position'] = start_pos
            
            # Convert relative trajectory to absolute based on current position and scale
            relative_trajectory = self.trajectories[selected_trajectory]
            absolute_trajectory = self.convert_to_absolute_trajectory(relative_trajectory, start_pos, scale)
            
            # Update trajectory points display
            points_text = ""
            for i, point in enumerate(absolute_trajectory):
                points_text += f"Point {i+1}: x={point[0]:.2f}, y={point[1]:.2f}, z={point[2]:.2f}\n"
            
            drone['widgets']['trajectory_points'].config(state='normal')
            drone['widgets']['trajectory_points'].delete(1.0, tki.END)
            drone['widgets']['trajectory_points'].insert(tki.END, points_text)
            drone['widgets']['trajectory_points'].config(state='disabled')
            
            drone['widgets']['current_point'].config(text="Starting...")
            
            rospy.loginfo(f"Starting trajectory {selected_trajectory} for drone {drone_id} "
                        f"from position: x={start_pos.x:.2f}, y={start_pos.y:.2f}, z={start_pos.z:.2f} "
                        f"with scale: {scale}")
            
            drone['data']['trajectory_active'] = True
            drone['data']['current_trajectory'] = absolute_trajectory
            drone['widgets']['start_trajectory'].configure(fg='green', bg='white')
            
            # Start trajectory thread
            thread = threading.Thread(target=self.execute_trajectory, args=(drone_id,))
            thread.daemon = True
            thread.start()

    def stop_trajectory(self, drone_id):
        if drone_id in self.drones:
            self.drones[drone_id]['data']['trajectory_active'] = False
            self.drones[drone_id]['widgets']['start_trajectory'].configure(fg='black', bg='yellow')
            self.drones[drone_id]['widgets']['current_point'].config(text="Trajectory stopped")

    def execute_trajectory(self, drone_id):
        drone = self.drones[drone_id]
        trajectory = drone['data']['current_trajectory']
        threshold = drone['trajectory_threshold']
        
        rospy.loginfo(f"Executing trajectory for drone {drone_id} with {len(trajectory)} points")
        
        # Publish full path
        path_msg = Path()
        path_msg.header.frame_id = 'world'
        path_msg.header.stamp = rospy.Time.now()
        
        for point in trajectory:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = point[0]
            pose_stamped.pose.position.y = point[1]
            pose_stamped.pose.position.z = point[2]
            
            # Convert yaw to quaternion
            q = quaternion_from_euler(0, 0, math.radians(point[3]))
            pose_stamped.pose.orientation.x = q[0]
            pose_stamped.pose.orientation.y = q[1]
            pose_stamped.pose.orientation.z = q[2]
            pose_stamped.pose.orientation.w = q[3]
            
            path_msg.poses.append(pose_stamped)
        
        drone['publishers']['path'].publish(path_msg)
        
        while drone['data']['trajectory_active'] and not self.quit_flag and trajectory:
            current_point = trajectory[0]
            current_pos = drone['data']['real_world_pos']
            
            # Update current point display
            point_index = len(self.trajectories[drone['widgets']['trajectory'].get()]) - len(trajectory)
            status_text = f"Executing Point {point_index + 1}: x={current_point[0]:.2f}, y={current_point[1]:.2f}, z={current_point[2]:.2f}"
            drone['widgets']['current_point'].config(text=status_text)
            
            # Create and publish command position
            command = Pose()
            command.position.x = current_point[0]
            command.position.y = current_point[1]
            command.position.z = current_point[2]
            
            # Convert yaw to quaternion
            q = quaternion_from_euler(0, 0, math.radians(current_point[3]))
            command.orientation.x = q[0]
            command.orientation.y = q[1]
            command.orientation.z = q[2]
            command.orientation.w = q[3]
            
            # Publish command multiple times to ensure it's received
            for _ in range(3):
                drone['publishers']['command_pos'].publish(command)
                rospy.sleep(0.1)
            
            # Log current status
            rospy.loginfo(f"Drone {drone_id} - Target: ({current_point[0]:.2f}, {current_point[1]:.2f}, {current_point[2]:.2f})")
            rospy.loginfo(f"Current: ({current_pos.x:.2f}, {current_pos.y:.2f}, {current_pos.z:.2f})")
            
            # Check if we've reached the current point
            if (abs(current_pos.x - current_point[0]) < threshold.x and
                abs(current_pos.y - current_point[1]) < threshold.y and
                abs(current_pos.z - current_point[2]) < threshold.z):
                rospy.loginfo(f"Reached point {len(trajectory)} for drone {drone_id}")
                trajectory.pop(0)
            else:
                rospy.loginfo(f"Distance to target - x: {abs(current_pos.x - current_point[0]):.2f}, "
                            f"y: {abs(current_pos.y - current_point[1]):.2f}, "
                            f"z: {abs(current_pos.z - current_point[2]):.2f}")
            
            rospy.sleep(0.5)
        # Reset display when finished
        drone['widgets']['current_point'].config(text="Trajectory completed")
        
        rospy.loginfo(f"Trajectory finished for drone {drone_id}")
        drone['widgets']['start_trajectory'].configure(fg='black', bg='yellow')
        drone['data']['trajectory_active'] = False

    def takeoff(self, drone_id):
        if drone_id in self.drones:
            self.drones[drone_id]['publishers']['takeoff'].publish(Empty())
            self.drones[drone_id]['widgets']['takeoff'].configure(fg='green', bg='white')
            self.drones[drone_id]['widgets']['land'].configure(fg='black', bg='yellow')

    def land(self, drone_id):
        if drone_id in self.drones:
            self.stop_trajectory(drone_id)
            self.drones[drone_id]['publishers']['land'].publish(Empty())
            self.drones[drone_id]['widgets']['takeoff'].configure(fg='black', bg='yellow')
            self.drones[drone_id]['widgets']['land'].configure(fg='green', bg='white')

    def emergency_stop(self, drone_id):
        if drone_id in self.drones:
            self.stop_trajectory(drone_id)
            self.drones[drone_id]['publishers']['emergency'].publish(Empty())

    def onClose(self, *args):
        print("[INFO] closing...")
        self.quit_flag = True
        for drone_id in list(self.drones.keys()):
            try:
                self.stop_trajectory(drone_id)
                self.land(drone_id)
            except:
                pass
        rospy.sleep(1.0)
        self.root.quit()
        self.root.destroy()

if __name__ == '__main__':
    root = tki.Tk()
    app = MultiTelloUI(root)
    root.mainloop()