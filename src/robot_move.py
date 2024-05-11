#!/usr/bin/env python3

import rospy 
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

class controller_class(): 

    def angle_callback(self, topic_data: Odometry):
        """
        Callback function for orientation data.
        Extracts yaw angle from orientation data and updates self.ori_z.
        """
        pose = topic_data.pose.pose
        orientation = pose.orientation 
        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w], 'sxyz')
        self.ori_z = yaw 

    def dist_callback(self, topic_data: LaserScan): 
        """
        Callback function for distance data.
        Processes laser scan data to find minimum distances in different directions.
        """
        dist = topic_data.ranges

        # Replace zero distances with a non-zero value (2 in this case)
        for i, d in enumerate(dist):
            if d == 0:
                dist[i] = 2
        
        # Extract distances in different directions
        front_dist = dist[0:35] + dist[325:359]
        back_dist = dist[135:225]
        tilt_dist_left = dist[20:90]
        tilt_dist_right = dist[270:340]

        # Update minimum distances
        self.min_front_dist = min(front_dist)
        self.min_back_dist = min(back_dist)
        self.min_tilt_dist_left = min(tilt_dist_left)
        self.min_tilt_dist_right = min(tilt_dist_right)

    def order_callback(self, topic_data: Float64):
        """
        Callback function for receiving ordered orientation.
        Updates self.wanted_z with the desired orientation.
        """
        self.wanted_z = topic_data.data

    def __init__(self):
        """
        Initialize the Controller class.
        """
        self.node_name = "Position_controller"  # Node name
        pub_topic_name = "cmd_vel"  # Publisher topic name
        sub_topic_name1 = "odom"  # Subscriber topic name for orientation
        sub_topic_name2 = "scan"  # Subscriber topic name for laser scan
        sub_topic_name3 = "move_cmd"  # Subscriber topic name for move commands

        self.ctrl_c = False  # Flag for Ctrl+C
        self.wanted_z = 0  # Desired orientation
        self.stop_dist = 0.32  # Distance to stop
        self.adj_side_dist = 0.2  # Distance for adjusting sideways
        self.normal_speed = 0.2  # Normal linear speed
        self.tilt_speed = 1  # Tilt linear speed
        self.ori_z = 0  # Current orientation z
        self.min_front_dist = 0  # Minimum front distance
        self.min_back_dist = 0  # Minimum back distance
        self.min_tilt_dist_left = 0  # Minimum tilt distance left
        self.min_tilt_dist_right = 0  # Minimum tilt distance right

        rospy.init_node(self.node_name, anonymous=True)  # Initialize ROS node

        # ROS Publishers and Subscribers
        self.pub = rospy.Publisher(pub_topic_name, Twist, queue_size=10)  # Publisher for cmd_vel
        self.sub1 = rospy.Subscriber(sub_topic_name1, Odometry, self.angle_callback)  # Subscriber for orientation
        self.sub2 = rospy.Subscriber(sub_topic_name2, LaserScan, self.dist_callback)  # Subscriber for laser scan
        self.sub3 = rospy.Subscriber(sub_topic_name3, Float64, self.order_callback)  # Subscriber for move commands
        self.rate = rospy.Rate(10)  # Rate of ROS node execution

        rospy.on_shutdown(self.shutdownhook)  # Register shutdown hook

        rospy.loginfo(f"The '{self.node_name}' node is active...")  # Log node activation

    def shutdownhook(self): 
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.pub.publish(stop_msg)
        self.ctrl_c = True

    def main_loop(self):
        """
        Main control loop for the robot.
        """
        while not self.ctrl_c:
            # Initialize Twist message for velocity commands
            vel_cmd = Twist()

            # Calculate angular speed based on orientation difference
            angle_dif1 = self.wanted_z - self.ori_z
            angle_dif2 = (0 - math.pi - self.ori_z) - (math.pi - self.wanted_z)
            if abs(angle_dif1) <= abs(angle_dif2):
                anglelar_speed = angle_dif1  # rad/s
            else:
                anglelar_speed = angle_dif2  # rad/s

            # Modify linear speed based on front and back distances
            linear_speed_dist_front_modif = self.normal_speed * math.exp((self.stop_dist - self.min_front_dist) * 10)
            if 0.8 * self.normal_speed <= abs(linear_speed_dist_front_modif) <= 1.2 * self.normal_speed:
                linear_speed_dist_front_modif = self.normal_speed

            linear_speed_dist_back_modif = self.normal_speed * math.exp((self.stop_dist - self.min_back_dist) * 10)
            if 0.8 * self.normal_speed <= abs(linear_speed_dist_back_modif) <= 1.2 * self.normal_speed:
                linear_speed_dist_back_modif = self.normal_speed

            # Modify angular speed based on tilt distances
            anglelar_speed_left_modif = 0
            if self.min_tilt_dist_left < self.adj_side_dist:
                anglelar_speed_left_modif = self.tilt_speed * math.exp((self.adj_side_dist * 0.75 - self.min_tilt_dist_left) * 5)
            if abs(anglelar_speed_left_modif) < 0.2 * self.tilt_speed:
                anglelar_speed_left_modif = 0

            anglelar_speed_right_modif = 0
            if self.min_tilt_dist_right < self.adj_side_dist:
                anglelar_speed_right_modif = self.tilt_speed * math.exp((self.adj_side_dist * 0.75 - self.min_tilt_dist_right) * 5)
            if abs(anglelar_speed_right_modif) < 0.2 * self.tilt_speed:
                anglelar_speed_right_modif = 0

            # Calculate final linear and angular velocities
            vel_cmd.linear.x = self.normal_speed - linear_speed_dist_front_modif + linear_speed_dist_back_modif - (anglelar_speed / math.pi) * self.normal_speed  # m/s
            vel_cmd.angular.z = anglelar_speed * 2 + anglelar_speed_right_modif - anglelar_speed_left_modif
            vel_cmd.angular.z = vel_cmd.angular.z

            # Publish velocity commands
            print(vel_cmd.linear.x)
            self.pub.publish(vel_cmd)
            self.rate.sleep()


if __name__ == '__main__': 
    robot_control = controller_class() 
    try:
        robot_control.main_loop() 
    except rospy.ROSInterruptException:
        pass