#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, Int32
import cv2
import pygame
import numpy as np
from cv_bridge import CvBridge

class TelloKeyboardNode:
    """ Maintains the Tello display and moves it through the keyboard keys.
        Press escape key to quit.
        The controls are:
            - T: Takeoff
            - L: Land
            - Arrow keys: Forward, backward, left and right.
            - A and D: Counter clockwise and clockwise rotations (yaw)
            - W and S: Up and down.

        保持Tello画面显示并用键盘移动它
        按下ESC键退出
        操作说明：
            T：起飞
            L：降落
            方向键：前后左右
            A和D：逆时针与顺时针转向
            W和S：上升与下降

    """
    def __init__(self):
        rospy.init_node('tello_keyboard_node', anonymous=True)
        
        # Initialize Pygame
        pygame.init()
        pygame.display.set_caption("Tello video stream")
        self.screen = pygame.display.set_mode([960, 720])

        # ROS publishers
        self.cmd_vel_pub = rospy.Publisher('tello/cmd_vel', Twist, queue_size=10)
        self.takeoff_pub = rospy.Publisher('tello/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('tello/land', Empty, queue_size=1)

        # ROS subscribers
        rospy.Subscriber('tello/camera/image_raw', Image, self.image_callback)
        rospy.Subscriber('tello/battery', Int32, self.battery_callback)

        # Control variables
        self.speed = 50
        self.send_rc_control = False

        # CV Bridge
        self.bridge = CvBridge()

        # Set up the Pygame clock
        self.clock = pygame.time.Clock()

        # Store the latest image and battery info
        self.latest_image = None
        self.latest_battery = 0

    def run(self):
        rate = rospy.Rate(30)  # 30Hz
        while not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
                elif event.type == pygame.KEYDOWN:
                    self.keydown(event.key)
                elif event.type == pygame.KEYUP:
                    self.keyup(event.key)

            # Display frame using Pygame
            if self.latest_image is not None:
                frame = self.latest_image
                text = f"Battery: {self.latest_battery:.2f}%"
                cv2.putText(frame, text, (5, 720 - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame = np.rot90(frame)
                frame = np.flipud(frame)
                frame = pygame.surfarray.make_surface(frame)
                self.screen.blit(frame, (0, 0))
                pygame.display.update()

            self.clock.tick(30)
            rate.sleep()

        pygame.quit()

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def battery_callback(self, msg):
        self.latest_battery = msg.data

    def keydown(self, key):
        if self.send_rc_control:
            msg = Twist()
            if key == pygame.K_UP:
                msg.linear.x = self.speed
            elif key == pygame.K_DOWN:
                msg.linear.x = -self.speed
            elif key == pygame.K_LEFT:
                msg.linear.y = -self.speed
            elif key == pygame.K_RIGHT:
                msg.linear.y = self.speed
            elif key == pygame.K_w:
                msg.linear.z = self.speed
            elif key == pygame.K_s:
                msg.linear.z = -self.speed
            elif key == pygame.K_a:
                msg.angular.z = -self.speed
            elif key == pygame.K_d:
                msg.angular.z = self.speed
            self.cmd_vel_pub.publish(msg)

    def keyup(self, key):
        if key in (pygame.K_UP, pygame.K_DOWN, pygame.K_LEFT, pygame.K_RIGHT, 
                   pygame.K_w, pygame.K_s, pygame.K_a, pygame.K_d):
            msg = Twist()
            self.cmd_vel_pub.publish(msg)
        elif key == pygame.K_t:
            self.takeoff_pub.publish(Empty())
            self.send_rc_control = True
        elif key == pygame.K_l:
            self.land_pub.publish(Empty())
            self.send_rc_control = False

if __name__ == '__main__':
    try:
        node = TelloKeyboardNode()
        node.run()
    except rospy.ROSInterruptException:
        pass