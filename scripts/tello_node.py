#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3Stamped, TwistStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, Int32, String, Float32
from cv_bridge import CvBridge
from djitellopy.tello import Tello
import cv2
from dataclasses import dataclass

@dataclass
class FlightData:
    """Store the drone's flight data with correct units"""
    # Battery state
    battery_percent: int = 0                        # 0-100
    estimated_flight_time_remaining: float = 0.0    # seconds
    flight_time: float = 0.0                        # seconds

    # Position and velocity (all in meters or meters/second)
    east_speed: float = -1.0                        # m/s, -1 means invalid
    north_speed: float = -1.0                       # m/s, -1 means invalid
    ground_speed: float = -1.0                      # m/s, -1 means invalid
    altitude: float = -1.0                          # Height off the ground in meters

    # Equipment status
    equipment: int = 0                              # 0=ok, 21=unstable, 205=timer exceeded
    high_temperature: bool = False                  # Temperature warning flag

    # Flight mode
    flight_mode: int = 1                            # 1=ground, 6=hover, 11=taking off, 12=landing, 41=spinning up

    # IMU data
    pitch: float = 0.0                              # degrees
    roll: float = 0.0                               # degrees
    yaw: float = 0.0                                # degrees
    agx: float = 0.0                                # Acceleration in x axis
    agy: float = 0.0                                # Acceleration in y axis
    agz: float = 0.0                                # Acceleration in z axis

class TelloNode:
    # Flight mode constants
    FLIGHT_MODE_GROUND = 1
    FLIGHT_MODE_HOVER = 6
    FLIGHT_MODE_TAKING_OFF = 11
    FLIGHT_MODE_LANDING = 12
    FLIGHT_MODE_SPINNING_UP = 41

    # Equipment status constants
    EQUIPMENT_OK = 0
    EQUIPMENT_UNSTABLE = 21
    EQUIPMENT_TIMER_EXCEEDED = 205
    
    def __init__(self):
        rospy.init_node('tello_node', anonymous=True)
        
        self.tello = Tello()
        self.tello.connect()
        rospy.loginfo("Connected to Tello successfully")
        
        self.bridge = CvBridge()
        
        try:
            self.id = rospy.get_param('~ID', '')
        except KeyError:
            self.id = ''
        self.publish_prefix = f"tello{self.id}/"
        
        self.flight_data = FlightData()
        self.is_flying = False
        
        self.init_publishers()
        self.init_subscribers()
        rospy.Timer(rospy.Duration(0.1), self.update_callback)

    def init_publishers(self):
        """Initialize all ROS topic publishers"""
        self.pubs = {
            'image': rospy.Publisher(self.publish_prefix +'camera/image_raw', Image, queue_size=10),
            'image_gray': rospy.Publisher(self.publish_prefix +'camera/image_gray', Image, queue_size=10),
            'altitude': rospy.Publisher(self.publish_prefix +'altitude', Float32, queue_size=10),
            'battery': rospy.Publisher(self.publish_prefix +'battery', Int32, queue_size=10),
            'flight_data': rospy.Publisher(self.publish_prefix +'flight_data', String, queue_size=10),
            'attitude': rospy.Publisher(self.publish_prefix +'flight/attitude', Vector3Stamped, queue_size=10),
            'velocity': rospy.Publisher(self.publish_prefix +'flight/velocity', TwistStamped, queue_size=10)
        }

    def init_subscribers(self):
        """Initialize all ROS topic subscribers"""
        rospy.Subscriber('tello/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('tello/takeoff', Empty, self.takeoff_callback)
        rospy.Subscriber('tello/land', Empty, self.land_callback)
        rospy.Subscriber('tello/emergency', Empty, self.emergency_callback)

    def update_flight_data(self):
        """Update all flight data from Tello with correct units"""
        # Get basic flight data
        self.flight_data.battery_percent = self.tello.get_battery()
        height_cm = self.tello.get_height()
        self.flight_data.altitude = -1.0 if height_cm == -1 else height_cm / 100.0  # Convert cm to meters
        self.flight_data.flight_time = self.tello.get_flight_time()
        
        # Get velocity data (already in correct units from SDK)
        self.flight_data.north_speed = self.tello.get_speed_x()
        self.flight_data.east_speed = self.tello.get_speed_y()
        
        # Get attitude data (already in degrees from SDK)
        self.flight_data.pitch = self.tello.get_pitch()
        self.flight_data.roll = self.tello.get_roll()
        self.flight_data.yaw = self.tello.get_yaw()
        
        # Get acceleration data
        self.flight_data.agx = self.tello.get_acceleration_x()
        self.flight_data.agy = self.tello.get_acceleration_y()
        self.flight_data.agz = self.tello.get_acceleration_z()

    def publish_flight_data(self):
        """Publish flight data to ROS topics with correct units"""
        now = rospy.Time.now()

        # Publish altitude in meters
        altitude_msg = Float32()
        altitude_msg.data = self.flight_data.altitude
        self.pubs['altitude'].publish(altitude_msg)

        # Publish battery percentage
        battery_msg = Int32()
        battery_msg.data = self.flight_data.battery_percent
        self.pubs['battery'].publish(battery_msg)
        
        # Publish attitude data (in degrees)
        attitude_msg = Vector3Stamped()
        attitude_msg.header.stamp = now
        attitude_msg.vector.x = self.flight_data.roll
        attitude_msg.vector.y = self.flight_data.pitch
        attitude_msg.vector.z = self.flight_data.yaw
        self.pubs['attitude'].publish(attitude_msg)

        # Publish velocity data (in m/s)
        velocity_msg = TwistStamped()
        velocity_msg.header.stamp = now
        velocity_msg.twist.linear.x = self.flight_data.north_speed
        velocity_msg.twist.linear.y = self.flight_data.east_speed
        velocity_msg.twist.linear.z = 0.0
        self.pubs['velocity'].publish(velocity_msg)

        # Publish comprehensive status information
        status_msg = (
            f"Battery: {self.flight_data.battery_percent}%, "
            f"Height: {self.flight_data.altitude:.2f}m, "
            f"Flight time: {self.flight_data.flight_time}s, "
            f"Mode: {self.get_flight_mode_string()}"
        )
        self.pubs['flight_data'].publish(status_msg)
        rospy.loginfo_throttle(5, status_msg)  # Log status every 5 seconds

    def get_flight_mode_string(self):
        """Convert flight mode number to descriptive string"""
        mode_strings = {
            self.FLIGHT_MODE_GROUND: "Ground",
            self.FLIGHT_MODE_HOVER: "Hover",
            self.FLIGHT_MODE_TAKING_OFF: "Taking Off",
            self.FLIGHT_MODE_LANDING: "Landing",
            self.FLIGHT_MODE_SPINNING_UP: "Spinning Up"
        }
        return mode_strings.get(self.flight_data.flight_mode, "Unknown")

    def update_camera(self):
        """Update and publish both RGB and grayscale camera data"""
        if not self.tello.stream_on:
            return
            
        frame = self.tello.get_frame_read().frame
        if frame is not None:
            try:
                # Publish RGB image
                rgb_msg = self.bridge.cv2_to_imgmsg(frame, "rgb8")
                self.pubs['image'].publish(rgb_msg)
                
                # Convert to grayscale and publish
                gray_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                gray_msg = self.bridge.cv2_to_imgmsg(gray_frame, "mono8")
                self.pubs['image_gray'].publish(gray_msg)
            except Exception as e:
                rospy.logerr(f"Camera error: {str(e)}")

    def update_callback(self, event):
        """Unified update callback function"""
        try:
            self.update_flight_data()
            self.publish_flight_data()
            self.update_camera()
        except Exception as e:
            rospy.logerr(f"Update error: {str(e)}")

    def cmd_vel_callback(self, msg):
        """Handle velocity control commands
        Converts incoming -1 to 1 range to -100 to 100 range for Tello SDK
        """
        if self.is_flying:
            self.tello.send_rc_control(
                int(msg.linear.y * 100),   # left/right velocity (-100 to 100)
                int(msg.linear.x * 100),   # forward/backward velocity (-100 to 100)
                int(msg.linear.z * 100),   # up/down velocity (-100 to 100)
                int(msg.angular.z * 100)   # yaw velocity (-100 to 100)
            )


    def takeoff_callback(self, _):
        """Handle takeoff command"""
        if not self.is_flying:
            try:
                self.tello.takeoff()
                self.is_flying = True
                rospy.loginfo("Takeoff successful")
            except Exception as e:
                rospy.logerr(f"Takeoff failed: {str(e)}")

    def land_callback(self, _):
        """Handle landing command"""
        if self.is_flying:
            try:
                self.tello.land()
                self.is_flying = False
                rospy.loginfo("Landing successful")
            except Exception as e:
                rospy.logerr(f"Landing failed: {str(e)}")

    def emergency_callback(self, _):
        """Handle emergency stop command"""
        try:
            self.tello.emergency()
            self.is_flying = False
            rospy.loginfo("Emergency stop executed")
        except Exception as e:
            rospy.logerr(f"Emergency stop failed: {str(e)}")

    def run(self):
        """Run the node"""
        self.tello.streamon()
        rospy.loginfo("Tello node is running")
        rospy.spin()

    def cleanup(self):
        """Clean up resources"""
        if self.is_flying:
            try:
                self.tello.land()
            except:
                self.tello.emergency()
        
        if self.tello.stream_on:
            self.tello.streamoff()
        self.tello.end()
        rospy.loginfo("Cleanup completed")

if __name__ == '__main__':
    tello_node = TelloNode()
    try:
        tello_node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        tello_node.cleanup()