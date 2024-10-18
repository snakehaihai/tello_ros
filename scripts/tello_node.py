#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, Int32, String
from cv_bridge import CvBridge
from djitellopy.tello import Tello
import cv2

class TelloNode:
    def __init__(self):
        rospy.init_node('tello_node', anonymous=True)
        
        self.tello = Tello()
        rospy.loginfo("Connecting to Tello...")
        self.tello.connect()
        rospy.loginfo("Connected to Tello successfully")

        rospy.loginfo("Starting video stream...")
        self.tello.streamon()
        rospy.loginfo("Video stream started")
        
        self.bridge = CvBridge()
        
        # Publishers
        self.image_pub = rospy.Publisher('tello/camera/image_raw', Image, queue_size=10)
        self.gray_image_pub = rospy.Publisher('tello/camera/image_gray', Image, queue_size=10)
        self.battery_pub = rospy.Publisher('tello/battery', Int32, queue_size=10)
        self.status_pub = rospy.Publisher('tello/status', String, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('tello/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('tello/takeoff', Empty, self.takeoff_callback)
        rospy.Subscriber('tello/land', Empty, self.land_callback)
        rospy.Subscriber('tello/flip', String, self.flip_callback)
        
        # Timer for publishing camera frames and battery status
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        self.is_flying = False
    
    def cmd_vel_callback(self, msg):
        if self.is_flying:
            self.tello.send_rc_control(
                int(msg.linear.y * 100),  # left_right_velocity
                int(msg.linear.x * 100),  # for_back_velocity
                int(msg.linear.z * 100),  # up_down_velocity
                int(msg.angular.z * 100)  # yaw_velocity
            )
            rospy.loginfo(f"Sending RC control: ({msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.linear.z:.2f}, {msg.angular.z:.2f})")
    
    def takeoff_callback(self, msg):
        if not self.is_flying:
            rospy.loginfo("Attempting takeoff...")
            self.tello.takeoff()
            self.is_flying = True
            rospy.loginfo("Takeoff successful")
    
    def land_callback(self, msg):
        if self.is_flying:
            rospy.loginfo("Attempting land...")
            self.tello.land()
            self.is_flying = False
            rospy.loginfo("Land successful")

    def flip_callback(self, msg):
        if self.is_flying:
            direction = msg.data.lower()
            if direction in ['f', 'b', 'l', 'r']:
                rospy.loginfo(f"Attempting flip in direction: {direction}")
                self.tello.flip(direction)
                rospy.loginfo("Flip successful")
    
    def timer_callback(self, event):
        # Publish camera frame
        frame = self.tello.get_frame_read().frame
        if frame is not None:
            # Publish RGB image
            rgb_image_msg = self.bridge.cv2_to_imgmsg(frame, "rgb8")
            self.image_pub.publish(rgb_image_msg)
            
            # Convert to grayscale and publish
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            gray_image_msg = self.bridge.cv2_to_imgmsg(gray_frame, "mono8")
            self.gray_image_pub.publish(gray_image_msg)
        
        # Publish status
        battery = self.tello.get_battery()
        height = self.tello.get_height()
        flight_time = self.tello.get_flight_time()
        status_msg = f"Battery: {battery}%, Height: {height}cm, Flight time: {flight_time}s"
        self.status_pub.publish(status_msg)
        self.battery_pub.publish(battery)
        rospy.loginfo_throttle(5, status_msg)  # Log status every 5 seconds

    def run(self):
        rospy.loginfo("Tello node is running")
        rospy.spin()
    
    def cleanup(self):
        rospy.loginfo("Cleaning up...")
        if self.is_flying:
            self.tello.land()
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