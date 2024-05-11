#!/usr/bin/env python3

import rospy 
import math
from std_msgs.msg import Float64 
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import random

class robot_handler(): 

    # Callback function to gather data from Odometry
    def angle_callback(self, odometry_data: Odometry):
        # Extract pose and orientation data from Odometry message
        pose = odometry_data.pose.pose
        orientation = pose.orientation 
        # Convert quaternion to Euler angles
        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w], 'sxyz')
        
        # Update orientation_z with yaw angle
        self.ori_z = yaw
        
        # Extract position data
        position = pose.position
        self.position_x = position.x
        self.position_y = position.y

    # Callback function to gather data from Lidar
    def dist_callback(self, laser_data: LaserScan): 
        # Extract distance data from LaserScan message
        distances = list(laser_data.ranges)
        
        # Remove zero distances
        for i in range(len(distances)):
            if distances[i] == 0:
                distances[i] = 2
                
        # Update lidar_data with the processed distances
        self.lidar_data = distances
        
        # Extract front, left, and right distances
        front_distances = distances[0:45] + distances[315:359]
        left_distances = distances[45:135]
        right_distances = distances[225:315]
        
        # Update minimum distances
        self.min_front_dist = min(front_distances)
        self.min_left_dist = min(left_distances)
        self.min_right_dist = min(right_distances)

    def __init__(self): 
        # Initialize node name and topic names
        self.node_name = "robot_handler" 
        pub_topic_name = "move_cmd"
        sub_topic_name1 = "odom"
        sub_topic_name2 = "scan"

        # Initialize control variables
        self.ctrl_c = False
        
        # Initialize variables for position, orientation, and obstacle detection
        self.ori_z = 0
        self.position_x = 0
        self.position_y = 0
        self.min_front_dist = 0
        self.min_right_dist = 0
        self.min_left_dist = 0
        self.stop_dist = 0.37
        self.obst_start_left_angle = 400
        self.obst_end_left_angle = 400
        self.obst_start_right_angle = 400
        self.obst_end_right_angle = 400
        self.left_obst_flag = False
        self.right_obst_flag = False
        self.left_angle_misplacement = 0
        self.right_angle_misplacement = 0
        self.lidar_data = []
        self.reversed_lidar_data = []
        self.obj_loc = 0                # 0: No Object , 1: Front , 2: Right , 3: Left , 4: Front Right , 5: Front Left , 6 : Right Left     
        self.prev_obj_loc = 0           # Previous object location
        self.prev_angle = 0             # Previous angle
        self.RL_dif = 0                 # Difference between right and left
        
        # Initialize ROS node and publishers/subscribers
        rospy.init_node(self.node_name, anonymous=True) 
        self.pub = rospy.Publisher(pub_topic_name, Float64, queue_size=10) 
        self.sub1 = rospy.Subscriber(sub_topic_name1, Odometry, self.angle_callback)
        self.sub2 = rospy.Subscriber(sub_topic_name2, LaserScan, self.dist_callback)
        self.rate = rospy.Rate(10) 

        # Shutdown hook
        rospy.on_shutdown(self.shutdownhook) 

        # Log node activity
        rospy.loginfo(f"The '{self.node_name}' node is active...") 

    def shutdownhook(self):
        # ROS shutdown handler
        self.ctrl_c = True
        rospy.loginfo("Shutting down...")

    def main_loop(self):
        while not self.ctrl_c: 
            out_angle = Float64()
            self.prev_obj_loc = self.obj_loc

            # Calculate obstacle angles from right and left
            self.detect_obstacle_angles()

            # Calculate misplacements between right and left angles
            self.calculate_angle_misplacement()

            # Detect objects
            self.detect_objects()

            # Motion handling
            self.handle_motion(out_angle)

            # Publish output angle
            print(out_angle)
            self.pub.publish(out_angle)
            self.rate.sleep()

    # Method to calculate obstacle angles from right and left
    def detect_obstacle_angles(self):
        if len(self.lidar_data) >= 45:
            for i in range(45):
                # Left front obstacle angle detection
                if not self.left_obst_flag:
                    if self.lidar_data[i] < self.stop_dist:
                        self.obst_start_left_angle = i
                        self.left_obst_flag = True
                else:
                    if self.lidar_data[i] > self.stop_dist:
                        self.obst_end_left_angle = i
                        self.left_obst_flag = False
                        break

                # Right front obstacle angle detection
                idx = len(self.lidar_data) - 1 - i
                if not self.right_obst_flag:
                    if self.lidar_data[idx] < self.stop_dist:
                        self.obst_start_right_angle = idx
                        self.right_obst_flag = True
                else:
                    if self.lidar_data[idx] > self.stop_dist:
                        self.obst_end_right_angle = idx
                        self.right_obst_flag = False
                        break
        else:
            rospy.logwarn("Lidar data length is less than 45. Cannot detect obstacle angles.")

    # Method to calculate misplacements between right and left angles
    def calculate_angle_misplacement(self):
        self.right_angle_misplacement = self.obst_end_right_angle - self.obst_start_right_angle
        self.left_angle_misplacement = self.obst_end_left_angle - self.obst_start_left_angle
        self.RL_dif = abs(self.right_angle_misplacement - self.left_angle_misplacement)

    # Method to detect objects
    def detect_objects(self):
        if self.min_front_dist <= self.stop_dist:
            if self.min_right_dist <= self.stop_dist:
                self.obj_loc = 4
            elif self.min_left_dist <= self.stop_dist:
                self.obj_loc = 5
            else: 
                self.obj_loc = 1
        elif self.min_right_dist <= self.stop_dist:
            self.obj_loc = 2
        elif self.min_left_dist <= self.stop_dist:
            self.obj_loc = 3
        else:
            self.obj_loc = 0

    # Method to handle motion
    def handle_motion(self, out_angle):
        if self.obj_loc == 0:
            out_angle.data = self.ori_z
        elif self.obj_loc == 1:
            # Handling front obstacle
            self.handle_front_obstacle(out_angle)
        elif self.obj_loc == 2:
            # Handling right obstacle
            out_angle.data = self.ori_z
        elif self.obj_loc == 3:
            # Handling left obstacle
            out_angle.data = self.ori_z
        elif self.obj_loc == 4:
            # Handling front right obstacle
            self.handle_front_right_obstacle(out_angle)
        elif self.obj_loc == 5:
            # Handling front left obstacle
            self.handle_front_left_obstacle(out_angle)
        elif self.obj_loc == 6:
            out_angle.data = self.ori_z

    # Method to handle front obstacle
    def handle_front_obstacle(self, out_angle):
        if self.RL_dif > 5:
            if 5 < self.left_angle_misplacement < 80:
                out_angle.data = self.ori_z - math.radians(self.left_angle_misplacement / 2)
            elif 5 < self.right_angle_misplacement < 80:
                if self.prev_angle < self.ori_z + math.radians(self.right_angle_misplacement / 2):
                    out_angle.data = self.ori_z + math.radians(self.right_angle_misplacement / 2)
            else:
                out_angle.data = self.ori_z + math.pi / 2 if self.prev_angle > self.ori_z else self.ori_z - math.pi / 2
        else:
            out_angle.data = self.ori_z + math.pi / 2 if self.prev_angle > self.ori_z else self.ori_z - math.pi / 2

    # Method to handle front right obstacle
    def handle_front_right_obstacle(self, out_angle):
        if self.right_angle_misplacement > 80:
            out_angle.data = self.ori_z + math.pi / 2
        else:
            if self.prev_angle < self.ori_z + math.radians(self.right_angle_misplacement / 2):
                out_angle.data = self.ori_z + math.radians(self.right_angle_misplacement / 2)

    # Method to handle front left obstacle
    def handle_front_left_obstacle(self, out_angle):
        if self.left_angle_misplacement > 80:
            out_angle.data = self.ori_z - math.pi / 2
        else:
            out_angle.data = self.ori_z - math.radians(self.left_angle_misplacement / 2)

if __name__ == '__main__':  
    robot_pos = robot_handler() 
    try:
        robot_pos.main_loop() 
    except rospy.ROSInterruptException:
        pass