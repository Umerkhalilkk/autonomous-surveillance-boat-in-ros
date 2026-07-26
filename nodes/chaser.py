#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import math
import time
from sensor_msgs.msg import Image, LaserScan, Imu, NavSatFix
from nav_msgs.msg import Odometry, Path # ADDED PATH HERE
from geometry_msgs.msg import Twist, PoseStamped # ADDED POSESTAMPED HERE
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion

class ChaseBoat:
    def __init__(self):
        rospy.init_node('boat_chaser')
        
        # --- CONFIGURATION ---
        self.LIDAR_STOP_DIST = 1.0      
        self.VISUAL_STOP_AREA = 35000   
        self.TOTAL_TARGETS = 2
        self.IGNORE_RADIUS = 3.5 
        
        self.HOME_X = -3.0
        self.HOME_Y = 0.0

        # Publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # NEW: Publisher for the green line
        self.path_pub = rospy.Publisher('/boat_path', Path, queue_size=10)
        
        # Subscribers
        self.image_sub = rospy.Subscriber('/boat/camera/image_raw', Image, self.image_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.gps_sub = rospy.Subscriber('/fix', NavSatFix, self.gps_callback)
        
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        self.bridge = CvBridge()
        self.cmd = Twist()
        
        # --- PATH VARIABLES (NEW) ---
        self.path = Path()
        self.path.header.frame_id = "odom" # Important: Must match your Odom frame
        
        # Variables
        self.target_visible = False
        self.target_center_x = 0
        self.image_width = 0
        self.target_area = 0   
        self.lidar_dist = 99.9
        self.visual_dist = 99.9 
        
        self.captured_count = 0 
        self.is_maneuvering = False 
        self.valid_lock = False 
        
        self.captured_locations = [] 
        self.current_yaw = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        
        # GPS Storage
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.cardinal_dir = "Unknown"

    # --- NEW GPS CALLBACK ---
    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def imu_callback(self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw
        
        deg = math.degrees(yaw)
        if -22.5 < deg <= 22.5: self.cardinal_dir = "EAST"
        elif 22.5 < deg <= 67.5: self.cardinal_dir = "NORTH-EAST"
        elif 67.5 < deg <= 112.5: self.cardinal_dir = "NORTH"
        elif 112.5 < deg <= 157.5: self.cardinal_dir = "NORTH-WEST"
        elif 157.5 < deg <= 180 or -180 <= deg <= -157.5: self.cardinal_dir = "WEST"
        elif -157.5 < deg <= -112.5: self.cardinal_dir = "SOUTH-WEST"
        elif -112.5 < deg <= -67.5: self.cardinal_dir = "SOUTH"
        elif -67.5 < deg <= -22.5: self.cardinal_dir = "SOUTH-EAST"

    def odom_callback(self, msg):
        # 1. Update Position
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y

        # 2. UPDATE THE PATH (Draw the line)
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)

    def scan_callback(self, msg):
        mid_index = len(msg.ranges) // 2
        window = 30 
        front_ranges = msg.ranges[mid_index-window : mid_index+window]
        valid_ranges = [r for r in front_ranges if r > 0.1 and r < 12.0]
        if valid_ranges:
            self.lidar_dist = min(valid_ranges)
        else:
            self.lidar_dist = 99.9

    def image_callback(self, msg):
        if self.is_maneuvering: return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_width = cv_image.shape[1]
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask1 = cv2.inRange(hsv, np.array([0, 50, 50]), np.array([15, 255, 255]))
            mask2 = cv2.inRange(hsv, np.array([160, 50, 50]), np.array([180, 255, 255]))
            mask = mask1 + mask2
            
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                self.target_area = cv2.contourArea(largest_contour)
                
                if self.target_area > 15: 
                    # Use LIDAR for distance if close, else use visual estimate
                    dist_to_use = self.lidar_dist if self.lidar_dist < 10.0 else (40.0 / math.sqrt(self.target_area))
                    
                    # Estimate World Position (Odom)
                    est_target_x = self.pos_x + (dist_to_use * math.cos(self.current_yaw))
                    est_target_y = self.pos_y + (dist_to_use * math.sin(self.current_yaw))
                    
                    # MEMORY CHECK
                    is_known = False
                    for (cx, cy) in self.captured_locations:
                        if math.sqrt((est_target_x - cx)**2 + (est_target_y - cy)**2) < self.IGNORE_RADIUS:
                            is_known = True
                            break
                    
                    if not is_known:
                        self.target_visible = True
                        self.valid_lock = True 
                        self.visual_dist = dist_to_use
                        M = cv2.moments(largest_contour)
                        if M['m00'] > 0:
                            self.target_center_x = int(M['m10'] / M['m00'])
                    else:
                        self.target_visible = False
                else:
                    self.target_visible = False
            else:
                self.target_visible = False
            cv2.imshow("BOAT CAMERA", cv_image); cv2.waitKey(1)
        except Exception as e: pass

    def timer_callback(self, event):
        self.navigate()

    def navigate(self):
        if self.captured_count >= self.TOTAL_TARGETS:
            self.return_to_base()
            return

        visual_stop = (self.target_visible and self.target_area > self.VISUAL_STOP_AREA)
        lidar_stop = (self.lidar_dist < self.LIDAR_STOP_DIST)

        if (lidar_stop or visual_stop) and self.valid_lock:
            # 1. Calculate Odom Coordinates (Original Logic)
            final_d = self.lidar_dist if self.lidar_dist < 5.0 else self.visual_dist
            target_x = self.pos_x + (final_d * math.cos(self.current_yaw))
            target_y = self.pos_y + (final_d * math.sin(self.current_yaw))
            
            # 2. NEW: Calculate True GPS Coordinates
            # Conversion: 1 meter approx 0.000009 degrees
            meter_to_deg = 0.000009
            lat_offset = (final_d * math.sin(self.current_yaw)) * meter_to_deg
            lon_offset = (final_d * math.cos(self.current_yaw)) * meter_to_deg
            
            target_lat = self.current_lat + lat_offset
            target_lon = self.current_lon + lon_offset

            self.captured_count += 1
            self.captured_locations.append((target_x, target_y))
            
            rospy.loginfo("========================================")
            rospy.loginfo(f"TARGET {self.captured_count}/{self.TOTAL_TARGETS} CAPTURED!")
            rospy.loginfo(f"DIRECTION: {self.cardinal_dir}")
            rospy.loginfo(f"ODOM LOC: X:{target_x:.2f}, Y:{target_y:.2f}")
            rospy.loginfo(f"GPS LOC : Lat:{target_lat:.6f}, Lon:{target_lon:.6f}") # PRINTING GPS
            rospy.loginfo("========================================")
            
            if self.captured_count < self.TOTAL_TARGETS:
                self.perform_evasive_maneuver()
            return

        if self.target_visible:
            error = (self.image_width / 2) - self.target_center_x
            self.cmd.angular.z = 0.003 * error
            self.cmd.linear.x = 0.5 
            rospy.loginfo_throttle(1, f"CHASING... GPS: {self.current_lat:.5f}")
        else:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.4
            rospy.loginfo_throttle(2, "SEARCHING...")

        self.cmd_pub.publish(self.cmd)

    def return_to_base(self):
        dx, dy = self.HOME_X - self.pos_x, self.HOME_Y - self.pos_y
        dist = math.sqrt(dx**2 + dy**2)
        if dist < 0.5:
            self.cmd.linear.x = 0.0; self.cmd.angular.z = 0.0
            rospy.loginfo_once("MISSION COMPLETE: AT HOME.")
        else:
            target_angle = math.atan2(dy, dx)
            error_yaw = target_angle - self.current_yaw
            while error_yaw > math.pi: error_yaw -= 2.0 * math.pi
            while error_yaw < -math.pi: error_yaw += 2.0 * math.pi
            self.cmd.angular.z = 0.8 * error_yaw 
            self.cmd.linear.x = 0.6 if abs(error_yaw) < 0.5 else 0.0
        self.cmd_pub.publish(self.cmd)

    def perform_evasive_maneuver(self):
        self.is_maneuvering = True 
        self.valid_lock = False 
        self.cmd.linear.x = -0.6; self.cmd.angular.z = 0.3
        self.cmd_pub.publish(self.cmd); rospy.sleep(4.0)
        self.target_visible = False; self.is_maneuvering = False 

if __name__ == '__main__':
    try:
        node = ChaseBoat(); rospy.spin()
    except rospy.ROSInterruptException: pass
