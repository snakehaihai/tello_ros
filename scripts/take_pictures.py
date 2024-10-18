#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Empty, EmptyResponse
import os
from datetime import datetime
import getpass

class TakePictureNode:
    """
    rosservice call /tello/take_picture
    """
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('take_picture_node', anonymous=True)
        
        self.bridge = CvBridge()
        self.image = None
        
        # Get the current username
        username = getpass.getuser()
        
        # Set the directory for saving pictures
        default_save_dir = f'/home/{username}/tello_pictures'
        self.save_dir = rospy.get_param('~save_dir', default_save_dir)
        
        # Create the save directory if it doesn't exist
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        
        # Set the image encoding to rgb8
        self.encoding = 'rgb8'
        
        rospy.loginfo(f"Pictures will be saved to: {self.save_dir}")
        rospy.loginfo(f"Using image encoding: {self.encoding}")
        
        # Subscribe to the image topic
        rospy.Subscriber('tello/camera/image_raw', Image, self.image_callback)
        
        # Create a service for taking pictures
        rospy.Service('tello/take_picture', Empty, self.take_picture_callback)
    
    def image_callback(self, msg):
        # Convert the ROS image message to OpenCV format
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.encoding)
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")
    
    def take_picture_callback(self, req):
        if self.image is not None:
            # Generate a filename with a timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"tello_picture_{timestamp}.png"
            filepath = os.path.join(self.save_dir, filename)
            
            try:
                # Save the image
                cv2.imwrite(filepath, self.image)
                rospy.loginfo(f"Picture saved as {filepath}")
            except Exception as e:
                rospy.logerr(f"Error saving image: {e}")
        else:
            rospy.logwarn("No image received yet")
        return EmptyResponse()
    
    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        node = TakePictureNode()
        node.run()
    except rospy.ROSInterruptException:
        pass