#!/usr/bin/env python3
import rospy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class MapLineFilter:
    def __init__(self):
        rospy.init_node('mapline_filter', anonymous=True)
        
        # Parameter configuration
        self.max_height = rospy.get_param('~max_height', 2.0)
        
        # Initialize publishers and subscribers
        self.pub = rospy.Publisher('/exp0/AirSLAM/filtered_maplines', Marker, queue_size=1)
        self.sub = rospy.Subscriber('/exp0/AirSLAM/mapline', Marker, self.callback, queue_size=1)
        
        # Initialize marker
        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.id = 0  # Add marker id
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = 0
        self.marker.pose.position.y = 0
        self.marker.pose.position.z = 0
        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0
        self.marker.pose.orientation.w = 1.0
        self.marker.type = Marker.LINE_LIST
        self.marker.scale.x = 0.05
        self.marker.color.b = 1.0
        self.marker.color.a = 1.0
        
        # Used to track the mapping of line ID to index
        self.mapline_id_to_index = {}
        
        rospy.loginfo(f"MapLine filter initialized with height threshold: {self.max_height}m")

    def generate_color(self, id):
        """Generate color (simplified to fixed blue here, consistent with the original code)"""
        color = ColorRGBA()
        color.r = 0.0
        color.g = 0.0
        color.b = 1.0
        color.a = 1.0
        return color

    def callback(self, msg):
        try:
            # Update timestamp
            self.marker.header.stamp = msg.header.stamp
            
            # Clear the point and color lists
            self.marker.points = []
            self.marker.colors = []
            
            # Process each line segment
            for i in range(0, len(msg.points), 2):
                p1 = msg.points[i]
                p2 = msg.points[i+1]
                
                # Check height threshold
                if max(p1.z, p2.z) <= self.max_height:
                    # Add points
                    self.marker.points.append(p1)
                    self.marker.points.append(p2)
                    
                    # Add color
                    line_id = i // 2  # Use index as ID
                    color = self.generate_color(line_id)
                    self.marker.colors.append(color)
                    self.marker.colors.append(color)
                    
                    # Save index mapping
                    self.mapline_id_to_index[line_id] = len(self.marker.points) - 2
            
            # Publish the filtered marker
            self.pub.publish(self.marker)
            
        except Exception as e:
            rospy.logerr(f"Error in callback: {e}")

if __name__ == '__main__':
    try:
        filter = MapLineFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass