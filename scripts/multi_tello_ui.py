#!/usr/bin/env python3

# This script provides a simple GUI to control multiple Tello drones using ROS
# It allows you to takeoff, land, and control drones using SLAM and relative trajectories
# The script uses Tkinter for the GUI and rospy for ROS communication
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
        
        # Define initial positions for each drones (xyz in m, yaw in degrees)
        self.initial_positions = {
            '0': {'x': 1.75, 'y': 1.35, 'z': 1.8, 'yaw': 30.0},
            '1': {'x': -0.60, 'y': 0.45, 'z': 1.8, 'yaw': -25.0},
            '2': {'x': 0.20, 'y': 0.45, 'z': 1.8, 'yaw': -25.0}
        }
        # Base scale for trajectories (can be adjusted by scale factor)
        self.base_scale = 1.0
        # Define relative trajectories (will be adjusted based on current position)
        self.trajectories = {
            'N': [
                [0.0, 0.0, 0.0, 0],     # Start (will be replaced with current position)
                [0.0, 0.0, 0.4, 0],     # Up
                [1.1, 0.0, 0.0, 0],    # N1
                [-1.1, -0.85, 0.0, 0],    # N2
                [1.1, 0.0, 0.0, 0],    # N3
                [0.0, 0.0, -0.4, 0],    # Back to start height
            ],
            'T': [
                [0.0, 0.0, 0.0, 0],     # Start
                [0.0, 0.0, 0.4, 0],     # Up
                [0.2, -0.85, 0.0, 0],    # T1
                [-0.1, 0.5, 0.0, 0],     # T2
                [-1.1, -0.1, 0.0, 0],    # T3
                [0.0, 0.0, -0.4, 0],    # Back to start height
            ],
            'U': [
                [0.0, 0.0, 0.0, 0],     # Start
                [0.0, 0.0, 0.4, 0],     # Up
                [-0.9, 0.0, 0.0, 0],     # U1
                [0.0, -0.65, 0.0, 0],    # U2
                [1.0, 0.0, 0.0, 0],     # U3
                [0.0, 0.0, -0.4, 0],  # Back to start
            ]
        }
        
        # Create main container frames
        self.control_frame = tki.Frame(root, relief=tki.SUNKEN)
        self.control_frame.grid(row=0, column=0, padx=5, pady=5)

        # Add batch control frame with adjusted layout
        self.batch_control_frame = tki.LabelFrame(root, text="Batch Controls", relief=tki.SUNKEN)
        self.batch_control_frame.grid(row=0, column=1, padx=5, pady=5, sticky='nsew')

        # Create two columns for batch controls
        left_column = tki.Frame(self.batch_control_frame)
        left_column.grid(row=0, column=0, padx=5, pady=5)

        right_column = tki.Frame(self.batch_control_frame)
        right_column.grid(row=0, column=1, padx=5, pady=5)

        # Left column buttons
        self.batch_takeoff_btn = tki.Button(left_column,
                                        text="Takeoff All",
                                        command=self.takeoff_all,
                                        bg='yellow',
                                        width=15)
        self.batch_takeoff_btn.grid(row=0, column=0, padx=5, pady=5)

        self.batch_land_btn = tki.Button(left_column,
                                    text="Land All",
                                    command=self.land_all,
                                    bg='white',
                                    width=15)
        self.batch_land_btn.grid(row=1, column=0, padx=5, pady=5)

        self.batch_emergency_btn = tki.Button(left_column,
                                            text="EMERGENCY ALL",
                                            command=self.emergency_all,
                                            bg='red',
                                            fg='white',
                                            width=15)
        self.batch_emergency_btn.grid(row=2, column=0, padx=5, pady=5)

        # Right column buttons
        self.batch_slam_btn = tki.Button(right_column,
                                    text="Enable All SLAM",
                                    command=self.toggle_all_slam_control,
                                    bg='yellow',
                                    width=15)
        self.batch_slam_btn.grid(row=0, column=0, padx=5, pady=5)

        self.batch_start_trajectory_btn = tki.Button(right_column,
                                                text="Start All Trajectories",
                                                command=self.start_all_trajectories,
                                                bg='yellow',
                                                width=15)
        self.batch_start_trajectory_btn.grid(row=1, column=0, padx=5, pady=5)

        self.batch_stop_trajectory_btn = tki.Button(right_column,
                                                text="Stop All Trajectories",
                                                command=self.stop_all_trajectories,
                                                bg='white',
                                                width=15)
        self.batch_stop_trajectory_btn.grid(row=2, column=0, padx=5, pady=5)
        
        self.batch_original_position_btn = tki.Button(right_column,
                                                text="Fly to Original Position",
                                                command=self.original_position,
                                                bg='white',
                                                width=15)
        self.batch_original_position_btn.grid(row=3, column=0, padx=5, pady=5)

        # Drone frames at the bottom
        self.drone_frames = tki.Frame(root, relief=tki.SUNKEN)
        self.drone_frames.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky='ew')

        # Add drone button in control frame
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
                'start_position': None,  # Store starting position for relative trajectories
                'orientation': Point(),  # Store orientation data
                'start_yaw': 0.0       # Store starting yaw for trajectories
            },
            'trajectory_threshold': Point(0.1, 0.1, 0.1)
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
        
        # Position display
        position_frame = tki.Frame(frame)
        position_frame.grid(row=3, column=0, columnspan=2, pady=5)
        
        tki.Label(position_frame, text="Position:").grid(row=0, column=0)
        widgets['position'] = tki.Label(position_frame, text="x: 0.0, y: 0.0, z: 0.0")
        widgets['position'].grid(row=0, column=1, padx=5)
        
        # Add orientation display
        tki.Label(position_frame, text="Yaw:").grid(row=0, column=2)
        widgets['orientation'] = tki.Label(position_frame, text="0.0°")
        widgets['orientation'].grid(row=0, column=3, padx=5)
        
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
        
        # Real world position
        rospy.Subscriber(prefix + 'real_world_pos', PoseStamped,
                        lambda msg: self.real_world_pos_callback(msg, drone_id))
        
        # Orientation
        rospy.Subscriber(prefix + 'orientation', Point,
                        lambda msg: self.orientation_callback(msg, drone_id))
        
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

    def convert_to_absolute_trajectory(self, relative_trajectory, start_pos, start_yaw, scale):
        """Convert relative trajectory to absolute based on starting position, yaw and scale"""
        absolute_trajectory = []
        current_pos = [start_pos.x, start_pos.y, start_pos.z, start_yaw]
        
        for point in relative_trajectory:
            # Scale x, y, z coordinates and add yaw to the starting yaw
            absolute_point = [
                current_pos[0] + point[0] * scale,
                current_pos[1] + point[1] * scale,
                current_pos[2] + point[2],
                start_yaw + point[3]  # Add relative yaw to starting yaw
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
        pos = msg.pose.position
        self.drones[drone_id]['data']['real_world_pos'] = pos
        self.drones[drone_id]['data']['has_position_data'] = True
        
        # Update position display
        self.drones[drone_id]['widgets']['position'].config(
            text=f"x: {pos.x:.2f}, y: {pos.y:.2f}, z: {pos.z:.2f}")
        
    def orientation_callback(self, msg, drone_id):
        if drone_id not in self.drones:
            return
        
        # Update orientation display (using z-axis rotation/yaw)
        self.drones[drone_id]['widgets']['orientation'].config(
            text=f"{msg.z:.1f}°")
        
        # Store the orientation data
        self.drones[drone_id]['data']['orientation'] = msg

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
        # Grab current position and orientation
        pos = drone['data']['real_world_pos']
        yaw_rad = math.radians(drone['data']['orientation'].z)  # Convert yaw from degrees to radians
        command = Pose()
        command.position.x = pos.x
        command.position.y = pos.y
        command.position.z = pos.z
        # Convert yaw (degrees) to quaternion
        q = quaternion_from_euler(0, 0, yaw_rad)
        command.orientation.x = q[0]
        command.orientation.y = q[1]
        command.orientation.z = q[2]
        command.orientation.w = q[3]
        # Publish command multiple times to ensure Tello stays in position
        for _ in range(3):
            drone['publishers']['command_pos'].publish(command)
            rospy.sleep(0.1)
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
            
            # Store current position and yaw as start position
            start_pos = drone['data']['real_world_pos']
            start_yaw = drone['data']['orientation'].z  # Get current yaw from orientation
            drone['data']['start_position'] = start_pos
            drone['data']['start_yaw'] = start_yaw
            
            # Convert relative trajectory to absolute based on current position, yaw and scale
            relative_trajectory = self.trajectories[selected_trajectory]
            absolute_trajectory = self.convert_to_absolute_trajectory(
                relative_trajectory, start_pos, start_yaw, scale)
            
            # Update trajectory points display
            points_text = ""
            for i, point in enumerate(absolute_trajectory):
                points_text += f"Point {i+1}: x={point[0]:.2f}, y={point[1]:.2f}, z={point[2]:.2f}, yaw={point[3]:.1f}°\n"
            
            drone['widgets']['trajectory_points'].config(state='normal')
            drone['widgets']['trajectory_points'].delete(1.0, tki.END)
            drone['widgets']['trajectory_points'].insert(tki.END, points_text)
            drone['widgets']['trajectory_points'].config(state='disabled')
            
            drone['widgets']['current_point'].config(text="Starting...")
            
            rospy.loginfo(f"Starting trajectory {selected_trajectory} for drone {drone_id} "
                        f"from position: x={start_pos.x:.2f}, y={start_pos.y:.2f}, z={start_pos.z:.2f} "
                        f"yaw={start_yaw:.1f}° with scale: {scale}")
            
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
            
            # Convert yaw (degrees) to quaternion
            yaw_rad = math.radians(point[3])  # Convert yaw from degrees to radians
            q = quaternion_from_euler(0, 0, yaw_rad)
            pose_stamped.pose.orientation.x = q[0]
            pose_stamped.pose.orientation.y = q[1]
            pose_stamped.pose.orientation.z = q[2]
            pose_stamped.pose.orientation.w = q[3]
            
            path_msg.poses.append(pose_stamped)
        
        drone['publishers']['path'].publish(path_msg)
        
        while drone['data']['trajectory_active'] and not self.quit_flag and trajectory:
            current_point = trajectory[0]
            current_pos = drone['data']['real_world_pos']
            current_yaw = drone['data']['orientation'].z
            
            # Update current point display
            point_index = len(self.trajectories[drone['widgets']['trajectory'].get()]) - len(trajectory)
            status_text = f"Executing Point {point_index + 1}: x={current_point[0]:.2f}, y={current_point[1]:.2f}, z={current_point[2]:.2f}, yaw={current_point[3]:.1f}°"
            drone['widgets']['current_point'].config(text=status_text)
            
            # Create and publish command position
            command = Pose()
            command.position.x = current_point[0]
            command.position.y = current_point[1]
            command.position.z = current_point[2]
            
            # Convert yaw (degrees) to quaternion
            yaw_rad = math.radians(current_point[3])  # Convert yaw from degrees to radians
            q = quaternion_from_euler(0, 0, yaw_rad)
            command.orientation.x = q[0]
            command.orientation.y = q[1]
            command.orientation.z = q[2]
            command.orientation.w = q[3]
            
            # Publish command multiple times to ensure it's received
            for _ in range(3):
                drone['publishers']['command_pos'].publish(command)
                rospy.sleep(0.1)
            
            # Log current status
            rospy.loginfo(f"Drone {drone_id} - Target: ({current_point[0]:.2f}, {current_point[1]:.2f}, {current_point[2]:.2f}, {current_point[3]:.1f}°)")
            rospy.loginfo(f"Current: ({current_pos.x:.2f}, {current_pos.y:.2f}, {current_pos.z:.2f}, {current_yaw:.1f}°)")
            
            # Check if we've reached the current point (including yaw alignment)
            # Note: You might want to adjust the yaw threshold based on your needs
            yaw_threshold = 5.0  # 5 degrees threshold for yaw alignment
            if (abs(current_pos.x - current_point[0]) < threshold.x and
                abs(current_pos.y - current_point[1]) < threshold.y and
                abs(current_pos.z - current_point[2]) < threshold.z and
                abs(current_yaw - current_point[3]) < yaw_threshold):
                
                rospy.loginfo(f"Reached point {len(trajectory)} for drone {drone_id}")
                trajectory.pop(0)
            else:
                rospy.loginfo(f"Distance to target - x: {abs(current_pos.x - current_point[0]):.2f}, "
                            f"y: {abs(current_pos.y - current_point[1]):.2f}, "
                            f"z: {abs(current_pos.z - current_point[2]):.2f}, "
                            f"yaw: {abs(current_yaw - current_point[3]):.1f}°")
            
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

    def takeoff_all(self):
        """Command all drones to take off"""
        for drone_id in self.drones:
            self.takeoff(drone_id)
        self.batch_takeoff_btn.configure(fg='green', bg='white')
        self.batch_land_btn.configure(fg='black', bg='yellow')

    def land_all(self):
        """Command all drones to land"""
        for drone_id in self.drones:
            self.land(drone_id)
        self.batch_takeoff_btn.configure(fg='black', bg='yellow')
        self.batch_land_btn.configure(fg='green', bg='white')

    def emergency_all(self):
        """Emergency stop all drones"""
        for drone_id in self.drones:
            self.emergency_stop(drone_id)

    def start_all_trajectories(self):
        """Start trajectories for all drones"""
        for drone_id in self.drones:
            self.start_trajectory(drone_id)
        self.batch_start_trajectory_btn.configure(fg='green', bg='white')

    def stop_all_trajectories(self):
        """Stop trajectories for all drones"""
        for drone_id in self.drones:
            self.stop_trajectory(drone_id)
        self.batch_start_trajectory_btn.configure(fg='black', bg='yellow')
        
    def original_position(self):
        """All drones fly to their original position"""
        for drone_id in self.drones:
            drone = self.drones[drone_id]
            init_pos = self.initial_positions.get(drone_id, {
                'x': 0.0, 'y': 0.0, 'z': 1.4, 'yaw': -90.0
            })
            yaw_rad = math.radians(init_pos['yaw'])
            
            command = Pose()
            command.position.x = init_pos['x']
            command.position.y = init_pos['y'] 
            command.position.z = init_pos['z']
            q = quaternion_from_euler(0, 0, yaw_rad)
            command.orientation.x = q[0]
            command.orientation.y = q[1]
            command.orientation.z = q[2]
            command.orientation.w = q[3]
            for _ in range(3):
                drone['publishers']['command_pos'].publish(command)
                rospy.sleep(0.1)

    def toggle_all_slam_control(self):
        """Toggle SLAM control for all drones"""
        # Check if any drone has SLAM control enabled
        any_enabled = any(drone['data']['allow_slam_control'] for drone in self.drones.values())
        
        # Set new state based on current state
        new_state = not any_enabled
        
        for drone_id in self.drones:
            drone = self.drones[drone_id]
            drone['data']['allow_slam_control'] = new_state
            drone['publishers']['allow_slam_control'].publish(Bool(new_state))
            
            # Update button state
            if new_state:
                drone['widgets']['slam_control'].configure(text="Disable SLAM Control", 
                                                         bg='black', fg='yellow')
            else:
                drone['widgets']['slam_control'].configure(text="Enable SLAM Control", 
                                                         bg='yellow', fg='black')
            
            # Grab current position and orientation
            if new_state:
                pos = drone['data']['real_world_pos']
                yaw_rad = math.radians(drone['data']['orientation'].z)
                command = Pose()
                command.position.x = pos.x
                command.position.y = pos.y
                command.position.z = pos.z
                q = quaternion_from_euler(0, 0, yaw_rad)
                command.orientation.x = q[0]
                command.orientation.y = q[1]
                command.orientation.z = q[2]
                command.orientation.w = q[3]
                for _ in range(3):
                    drone['publishers']['command_pos'].publish(command)
                    rospy.sleep(0.1)
        
        # Update batch button text
        if new_state:
            self.batch_slam_btn.configure(text="Disable All SLAM", 
                                        bg='black', fg='yellow')
        else:
            self.batch_slam_btn.configure(text="Enable All SLAM", 
                                        bg='yellow', fg='black')
        
        rospy.loginfo(f"SLAM Control for all drones {'enabled' if new_state else 'disabled'}")

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