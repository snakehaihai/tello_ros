#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3Stamped, TwistStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, Int32, String, Float32
from cv_bridge import CvBridge
from djitellopy.tello import Tello
import cv2
from dataclasses import dataclass
import threading
import time
from typing import Dict

@dataclass
class FlightData:
    """Store the drone's flight data with correct units"""
    battery_percent: int = 0
    estimated_flight_time_remaining: float = 0.0
    flight_time: float = 0.0
    east_speed: float = -1.0
    north_speed: float = -1.0
    ground_speed: float = -1.0
    altitude: float = -1.0
    equipment: int = 0
    high_temperature: bool = False
    flight_mode: int = 1
    pitch: float = 0.0
    roll: float = 0.0
    yaw: float = 0.0
    agx: float = 0.0
    agy: float = 0.0
    agz: float = 0.0

class VideoStreamHandler:
    def __init__(self, tello, port):
        self.tello = tello
        self.port = port
        self.frame = None
        self.stopped = False
        self.retry_count = 0
        self.max_retries = 3
        self.retry_delay = 2
        self.lock = threading.Lock()
        
    def start(self):
        self.stopped = False
        threading.Thread(target=self._update_frame, daemon=True).start()
        return self
        
    def _update_frame(self):
        while not self.stopped:
            try:
                frame_read = self.tello.get_frame_read(port=self.port)
                while not self.stopped:
                    current_frame = frame_read.frame
                    if current_frame is not None:
                        with self.lock:
                            self.frame = current_frame.copy()
                    time.sleep(1/30)
            except Exception as e:
                rospy.logerr(f"Video stream error: {str(e)}")
                self.retry_count += 1
                if self.retry_count > self.max_retries:
                    rospy.logerr("Max retries exceeded for video stream")
                    self.stopped = True
                    break
                rospy.logwarn(f"Retrying video stream... ({self.retry_count}/{self.max_retries})")
                time.sleep(self.retry_delay)
                
    def get_frame(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None
            
    def stop(self):
        self.stopped = True

class TelloInstance:
    FLIGHT_MODE_GROUND = 1
    FLIGHT_MODE_HOVER = 6
    FLIGHT_MODE_TAKING_OFF = 11
    FLIGHT_MODE_LANDING = 12
    FLIGHT_MODE_SPINNING_UP = 41

    def __init__(self, tello_id: str, drone_ip: str, video_port: int):
        self.id = tello_id
        self.drone_ip = drone_ip
        self.video_port = video_port
        self.flight_data = FlightData()
        self.is_flying = False
        self.video_handler = None
        self.bridge = CvBridge()
        self.pubs = {}
        
        # 初始化 Tello
        self.tello = None
        self.connect_tello()
        
    def connect_tello(self):
        try:
            self.tello = Tello(self.drone_ip)
            self.tello.connect()
            # self.tello.set_video_port(self.video_port)
            rospy.loginfo(f"Connected to Tello {self.id} at {self.drone_ip}")
        except Exception as e:
            rospy.logerr(f"Failed to connect to Tello {self.id}: {str(e)}")
            raise

    def init_publishers(self, namespace: str):
        """Initialize ROS publishers for this Tello instance"""
        self.pubs = {
            'image': rospy.Publisher(f'{namespace}/camera/image_raw', Image, queue_size=10),
            'image_gray': rospy.Publisher(f'{namespace}/camera/image_gray', Image, queue_size=10),
            'altitude': rospy.Publisher(f'{namespace}/altitude', Float32, queue_size=10),
            'battery': rospy.Publisher(f'{namespace}/battery', Int32, queue_size=10),
            'flight_data': rospy.Publisher(f'{namespace}/flight_data', String, queue_size=10),
            'attitude': rospy.Publisher(f'{namespace}/flight/attitude', Vector3Stamped, queue_size=10),
            'velocity': rospy.Publisher(f'{namespace}/flight/velocity', TwistStamped, queue_size=10)
        }

    def start_video(self):
        """Start video streaming"""
        try:
            self.tello.streamon()
            self.video_handler = VideoStreamHandler(self.tello, self.video_port)
            self.video_handler.start()
        except Exception as e:
            rospy.logerr(f"Tello {self.id} video start error: {str(e)}")

    def update_flight_data(self):
        """Update all flight data from Tello"""
        try:
            self.flight_data.battery_percent = self.tello.get_battery()
            height_cm = self.tello.get_height()
            self.flight_data.altitude = -1.0 if height_cm == -1 else height_cm / 100.0
            self.flight_data.flight_time = self.tello.get_flight_time()
            self.flight_data.pitch = self.tello.get_pitch()
            self.flight_data.roll = self.tello.get_roll()
            self.flight_data.yaw = self.tello.get_yaw()
            self.flight_data.north_speed = self.tello.get_speed_x()
            self.flight_data.east_speed = self.tello.get_speed_y()
            self.flight_data.agx = self.tello.get_acceleration_x()
            self.flight_data.agy = self.tello.get_acceleration_y()
            self.flight_data.agz = self.tello.get_acceleration_z()
        except Exception as e:
            rospy.logerr(f"Tello {self.id} update flight data error: {str(e)}")

    def publish_data(self):
        """Publish all data to ROS topics"""
        try:
            now = rospy.Time.now()
            
            # Publish basic data
            self.pubs['altitude'].publish(Float32(self.flight_data.altitude))
            self.pubs['battery'].publish(Int32(self.flight_data.battery_percent))
            
            # Publish attitude
            attitude_msg = Vector3Stamped()
            attitude_msg.header.stamp = now
            attitude_msg.vector.x = self.flight_data.roll
            attitude_msg.vector.y = self.flight_data.pitch
            attitude_msg.vector.z = self.flight_data.yaw
            self.pubs['attitude'].publish(attitude_msg)
            
            # Publish velocity
            vel_msg = TwistStamped()
            vel_msg.header.stamp = now
            vel_msg.twist.linear.x = self.flight_data.north_speed
            vel_msg.twist.linear.y = self.flight_data.east_speed
            self.pubs['velocity'].publish(vel_msg)

            # Publish flight data summary
            status = (
                f"Battery: {self.flight_data.battery_percent}%, "
                f"Height: {self.flight_data.altitude:.2f}m, "
                f"Flight time: {self.flight_data.flight_time}s"
            )
            self.pubs['flight_data'].publish(status)
            
            # Update camera feed
            self.publish_camera()
            
        except Exception as e:
            rospy.logerr(f"Tello {self.id} publish data error: {str(e)}")

    def publish_camera(self):
        """Publish camera frames"""
        if self.video_handler:
            frame = self.video_handler.get_frame()
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
                    rospy.logerr(f"Tello {self.id} camera publish error: {str(e)}")

    def handle_cmd_vel(self, msg):
        """Handle velocity control command"""
        if self.is_flying:
            try:
                self.tello.send_rc_control(
                    int(msg.linear.y * 100),
                    int(msg.linear.x * 100),
                    int(msg.linear.z * 100),
                    int(msg.angular.z * 100)
                )
            except Exception as e:
                rospy.logerr(f"Tello {self.id} velocity command error: {str(e)}")

    def takeoff(self):
        """Execute takeoff command"""
        if not self.is_flying:
            try:
                self.tello.takeoff()
                self.is_flying = True
                rospy.loginfo(f"Tello {self.id} takeoff successful")
            except Exception as e:
                rospy.logerr(f"Tello {self.id} takeoff failed: {str(e)}")

    def land(self):
        """Execute landing command"""
        if self.is_flying:
            try:
                self.tello.land()
                self.is_flying = False
                rospy.loginfo(f"Tello {self.id} landing successful")
            except Exception as e:
                rospy.logerr(f"Tello {self.id} landing failed: {str(e)}")

    def emergency(self):
        """Execute emergency stop"""
        try:
            self.tello.emergency()
            self.is_flying = False
            rospy.loginfo(f"Tello {self.id} emergency stop executed")
        except Exception as e:
            rospy.logerr(f"Tello {self.id} emergency stop failed: {str(e)}")

    def cleanup(self):
        """Clean up resources"""
        try:
            if self.is_flying:
                self.land()
            if self.video_handler:
                self.video_handler.stop()
            if self.tello.stream_on:
                self.tello.streamoff()
            self.tello.end()
            rospy.loginfo(f"Tello {self.id} cleanup completed")
        except Exception as e:
            rospy.logerr(f"Tello {self.id} cleanup error: {str(e)}")

class MultiTelloNode:
    def __init__(self):
        rospy.init_node('multi_tello_node', anonymous=True)
        self.tello_instances: Dict[str, TelloInstance] = {}
        self.load_tello_configs()
        self.init_subscribers()
        self.update_timer = rospy.Timer(rospy.Duration(0.1), self.update_callback)

    def load_tello_configs(self):
        """Load Tello configurations from ROS parameters"""
        tello_configs = rospy.get_param('~tello_configs', [])
        for config in tello_configs:
            tello_id = config['id']
            drone_ip = config['ip']
            video_port = config['video_port']
            
            try:
                # 创建Tello实例
                tello = TelloInstance(tello_id, drone_ip, video_port)
                # 初始化发布者
                tello.init_publishers(f'/tello{tello_id}')
                # 启动视频流
                tello.start_video()
                # 存储实例
                self.tello_instances[tello_id] = tello
                rospy.loginfo(f"Initialized Tello {tello_id}")
            except Exception as e:
                rospy.logerr(f"Failed to initialize Tello {tello_id}: {str(e)}")

    def init_subscribers(self):
        """Initialize subscribers for each Tello"""
        for tello_id, tello in self.tello_instances.items():
            prefix = f'/tello{tello_id}'
            rospy.Subscriber(f'{prefix}/cmd_vel', Twist, 
                           lambda msg, t=tello: t.handle_cmd_vel(msg))
            rospy.Subscriber(f'{prefix}/takeoff', Empty,
                           lambda msg, t=tello: t.takeoff())
            rospy.Subscriber(f'{prefix}/land', Empty,
                           lambda msg, t=tello: t.land())
            rospy.Subscriber(f'{prefix}/emergency', Empty,
                           lambda msg, t=tello: t.emergency())

    def update_callback(self, event):
        """Update all Tello instances"""
        for tello in self.tello_instances.values():
            tello.update_flight_data()
            tello.publish_data()

    def cleanup(self):
        """Clean up all Tello instances"""
        for tello in self.tello_instances.values():
            tello.cleanup()

if __name__ == '__main__':
    node = None
    try:
        node = MultiTelloNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if node:
            node.cleanup()