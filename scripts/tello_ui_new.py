#!/usr/bin/env python3

import rospy
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from std_msgs.msg import String, Float32, Empty
from geometry_msgs.msg import Vector3Stamped, TwistStamped, Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from PIL import Image as PILImage
from PIL import ImageTk

class TelloViz:
    def __init__(self, root):
        self.root = root
        self.root.title("Tello Drone Monitor & Control")
        
        # Initialize data storage
        self.max_points = 100
        self.times = np.zeros(self.max_points)
        self.altitudes = np.zeros(self.max_points)
        self.current_index = 0
        self.start_time = rospy.Time.now()
        
        # Flight control variables
        self.is_flying = False
        self.speed_scale = 0.5  # Scale factor for speed (0.0 to 1.0)
        
        # Initialize bridge for camera
        self.bridge = CvBridge()
        
        self.setup_gui()
        self.setup_ros()
        
        # Keyboard bindings for control
        self.setup_keyboard_control()
        
        # Update GUI periodically
        self.root.after(100, self.update_gui)

    def setup_ros(self):
        """Setup ROS publishers and subscribers"""
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('tello/cmd_vel', Twist, queue_size=1)
        self.takeoff_pub = rospy.Publisher('tello/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('tello/land', Empty, queue_size=1)
        self.emergency_pub = rospy.Publisher('tello/emergency', Empty, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('tello/flight_data', String, self.flight_data_callback)
        rospy.Subscriber('tello/altitude', Float32, self.altitude_callback)
        rospy.Subscriber('tello/flight/attitude', Vector3Stamped, self.attitude_callback)
        rospy.Subscriber('tello/camera/image_raw', Image, self.image_callback)

    def setup_gui(self):
        """Setup the GUI elements"""
        # Main container
        main_container = ttk.Frame(self.root, padding="5")
        main_container.grid(row=0, column=0, sticky="nsew")
        
        # Status Frame
        status_frame = self.create_status_frame(main_container)
        status_frame.grid(row=0, column=0, columnspan=2, sticky="ew")
        
        # Control Frame
        control_frame = self.create_control_frame(main_container)
        control_frame.grid(row=1, column=0, sticky="nsew")
        
        # Plot and Camera Frame
        viz_frame = self.create_visualization_frame(main_container)
        viz_frame.grid(row=1, column=1, sticky="nsew")
        
        # Attitude Frame
        attitude_frame = self.create_attitude_frame(main_container)
        attitude_frame.grid(row=2, column=0, columnspan=2, sticky="ew")
        
        # Configure grid weights
        main_container.grid_columnconfigure(1, weight=1)
        main_container.grid_rowconfigure(1, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(0, weight=1)

    def create_status_frame(self, parent):
        """Create status indicators"""
        frame = ttk.LabelFrame(parent, text="Status", padding="5")
        
        self.battery_label = ttk.Label(frame, text="Battery: ---%")
        self.battery_label.pack(side=tk.LEFT, padx=5)
        
        self.altitude_label = ttk.Label(frame, text="Altitude: --- m")
        self.altitude_label.pack(side=tk.LEFT, padx=5)
        
        self.mode_label = ttk.Label(frame, text="Mode: ---")
        self.mode_label.pack(side=tk.LEFT, padx=5)
        
        return frame

    def create_control_frame(self, parent):
        """Create flight control panel"""
        frame = ttk.LabelFrame(parent, text="Flight Controls", padding="5")
        
        # Takeoff & Land buttons
        btn_frame = ttk.Frame(frame)
        btn_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Button(btn_frame, text="Takeoff", command=self.takeoff).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Land", command=self.land).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Emergency Stop", command=self.emergency_stop).pack(side=tk.LEFT, padx=5)
        
        # Speed control
        speed_frame = ttk.Frame(frame)
        speed_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(speed_frame, text="Speed:").pack(side=tk.LEFT)
        self.speed_scale_widget = ttk.Scale(speed_frame, from_=0.1, to=1.0, orient=tk.HORIZONTAL)
        self.speed_scale_widget.set(0.5)
        self.speed_scale_widget.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        # Control instructions
        instruction_text = """
        Control Keys:
        ↑: Forward    ↓: Backward
        ←: Left       →: Right
        W: Up         S: Down
        A: Yaw Left   D: Yaw Right
        Space: Stop Movement
        ESC: Emergency Stop
        """
        ttk.Label(frame, text=instruction_text, justify=tk.LEFT).pack(padx=5, pady=5)
        
        return frame

    def create_visualization_frame(self, parent):
        """Create visualization panel with plot and camera feed"""
        frame = ttk.Frame(parent)
        
        # Setup matplotlib figure for plotting
        self.fig, self.ax = plt.subplots(figsize=(6, 3))
        self.canvas = FigureCanvasTkAgg(self.fig, master=frame)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
        # Camera feed
        self.camera_label = ttk.Label(frame)
        self.camera_label.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)
        
        return frame

    def create_attitude_frame(self, parent):
        """Create attitude indicators"""
        frame = ttk.LabelFrame(parent, text="Attitude", padding="5")
        
        self.roll_label = ttk.Label(frame, text="Roll: 0.0°")
        self.roll_label.pack(side=tk.LEFT, padx=5)
        
        self.pitch_label = ttk.Label(frame, text="Pitch: 0.0°")
        self.pitch_label.pack(side=tk.LEFT, padx=5)
        
        self.yaw_label = ttk.Label(frame, text="Yaw: 0.0°")
        self.yaw_label.pack(side=tk.LEFT, padx=5)
        
        return frame

    def setup_keyboard_control(self):
        """Setup keyboard bindings for flight control"""
        self.root.bind('<KeyPress>', self.key_press)
        self.root.bind('<KeyRelease>', self.key_release)
        
        # Key states
        self.pressed_keys = set()
        
        # Control mapping
        self.control_keys = {
            'Up': (1, 0, 0, 0),     # forward
            'Down': (-1, 0, 0, 0),   # backward
            'Left': (0, -1, 0, 0),   # left
            'Right': (0, 1, 0, 0),   # right
            'w': (0, 0, 1, 0),       # up
            's': (0, 0, -1, 0),      # down
            'a': (0, 0, 0, -1),      # yaw left
            'd': (0, 0, 0, 1),       # yaw right
        }

    def key_press(self, event):
        """Handle key press events"""
        key = event.keysym
        if key not in self.pressed_keys:
            self.pressed_keys.add(key)
            self.update_drone_velocity()

    def key_release(self, event):
        """Handle key release events"""
        key = event.keysym
        if key in self.pressed_keys:
            self.pressed_keys.remove(key)
            self.update_drone_velocity()

    def update_drone_velocity(self):
        """Update drone velocity based on pressed keys"""
        if not self.is_flying:
            return

        # Initialize velocity components
        fb, lr, ud, yaw = 0, 0, 0, 0
        
        # Combine all active controls
        for key in self.pressed_keys:
            if key in self.control_keys:
                vel = self.control_keys[key]
                fb += vel[0]
                lr += vel[1]
                ud += vel[2]
                yaw += vel[3]

        # Create and publish velocity command
        cmd = Twist()
        speed = self.speed_scale_widget.get()
        cmd.linear.x = fb * speed
        cmd.linear.y = lr * speed
        cmd.linear.z = ud * speed
        cmd.angular.z = yaw * speed
        
        self.cmd_vel_pub.publish(cmd)

    def takeoff(self):
        """Command drone to takeoff"""
        if not self.is_flying:
            self.takeoff_pub.publish(Empty())
            self.is_flying = True

    def land(self):
        """Command drone to land"""
        if self.is_flying:
            self.land_pub.publish(Empty())
            self.is_flying = False

    def emergency_stop(self):
        """Emergency stop command"""
        self.emergency_pub.publish(Empty())
        self.is_flying = False

    def update_plot(self):
        """Update the matplotlib plot"""
        self.ax.clear()
        valid_data = self.current_index if self.current_index < self.max_points else self.max_points
        self.ax.plot(self.times[:valid_data], self.altitudes[:valid_data], 'b-', label='Altitude (m)')
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Altitude (m)')
        self.ax.set_title('Flight Data')
        self.ax.grid(True)
        self.ax.legend()
        self.canvas.draw()

    def update_gui(self):
        """Periodic GUI update"""
        self.update_plot()
        self.root.after(100, self.update_gui)

    def flight_data_callback(self, msg):
        """Handle flight data updates"""
        data = msg.data.split(',')
        for item in data:
            if 'Battery' in item:
                battery = float(item.split(':')[1].strip().replace('%', ''))
                self.battery_label.config(text=f'Battery: {battery:.1f}%')
            elif 'Mode' in item:
                mode = item.split(':')[1].strip()
                self.mode_label.config(text=f'Mode: {mode}')

    def altitude_callback(self, msg):
        """Handle altitude updates"""
        altitude = msg.data
        self.altitude_label.config(text=f'Altitude: {altitude:.2f} m')
        
        # Update plot data
        if self.current_index >= self.max_points:
            # Shift arrays left
            self.times[:-1] = self.times[1:]
            self.altitudes[:-1] = self.altitudes[1:]
            self.current_index = self.max_points - 1
            
        self.times[self.current_index] = (rospy.Time.now() - self.start_time).to_sec()
        self.altitudes[self.current_index] = altitude
        self.current_index += 1

    def attitude_callback(self, msg):
        """Handle attitude updates"""
        self.roll_label.config(text=f'Roll: {msg.vector.x:.1f}°')
        self.pitch_label.config(text=f'Pitch: {msg.vector.y:.1f}°')
        self.yaw_label.config(text=f'Yaw: {msg.vector.z:.1f}°')

    def image_callback(self, msg):
        """Handle camera image updates"""
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            
            # Resize image if needed
            height, width = cv_image.shape[:2]
            max_size = 320
            if height > max_size or width > max_size:
                scale = max_size / max(height, width)
                cv_image = cv2.resize(cv_image, (int(width * scale), int(height * scale)))
            
            # Convert to PIL Image and then to PhotoImage
            pil_image = PILImage.fromarray(cv_image)
            photo = ImageTk.PhotoImage(image=pil_image)
            
            # Update label
            self.camera_label.config(image=photo)
            self.camera_label.image = photo  # Keep a reference
            
        except Exception as e:
            rospy.logerr(f"Image conversion error: {str(e)}")

def main():
    rospy.init_node('tello_visualization')
    
    root = tk.Tk()
    app = TelloViz(root)
    
    # Handle window close
    def on_closing():
        if app.is_flying:
            app.land()
        root.quit()
    root.protocol("WM_DELETE_WINDOW", on_closing)
    
    try:
        root.mainloop()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()