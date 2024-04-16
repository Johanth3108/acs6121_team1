#!/usr/bin/env python3
# A simple ROS publisher node in Python

# Importing required ROS libraries and message types
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import rospkg
import time
import subprocess

# Define the Explorer class
class Explorer:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('obstacle_avoidance_node')
        
        # Publisher for sending velocity commands
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        
        # Subscriber for receiving laser scan data
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback_function)
        
        # Flags for controlling startup and shutdown
        self.startup = True
        self.ctrl_c = False
        
        # Flag for controlling robot movement (turning or moving forward)
        self.turn = False
        
        # Rate object for controlling loop frequency
        self.rate = rospy.Rate(10)  # Hz
        
        # Twist object for setting robot velocities
        self.vel = Twist()
        
        # Register shutdown hook
        rospy.on_shutdown(self.shutdownhook)

    # Callback function for processing laser scan data
    def callback_function(self, dt):
        # Define obstacle avoidance thresholds
        thr1 = 0.8
        thr2 = 0.8
        # Initialize Twist message
        self.move = Twist()
        # Check for obstacles in front and slightly to the left and right
        if dt.ranges[0] > thr1 and dt.ranges[15] > thr2 and dt.ranges[345] > thr2:
            # Move forward if no obstacles detected
            self.move.linear.x = 0.1
            self.move.angular.z = 0.0
        else:
            # Stop and rotate if obstacles detected
            self.move.linear.x = 0.0
            self.move.angular.z = 0.5

    # Shutdown hook to stop the robot on shutdown
    def shutdownhook(self):
        self.pub.publish(Twist())
        self.ctrl_c = True

    # Function to save the map
    def save_map(self):
        rospy.loginfo("Saving map...")
        # Initialize ROS package manager
        rospack = rospkg.RosPack()
        # Get path to package
        package_path = rospack.get_path('acs6121_team1')
        # Define path to map folder
        map_folder_path = package_path + '/map/map1'
        try:
            # Delay to ensure map is fully generated
            time.sleep(5)
            # Save map using map_server package
            subprocess.run('rosrun map_server map_saver -f ' + map_folder_path, shell=True)
            subprocess.Popen('echo test', shell=True)
            rospy.logwarn("Map saved successfully to " + map_folder_path)
            # Additional delay for safety
            time.sleep(5)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to save map: %s" % e)

    # Main loop for controlling robot movement
    def main_loop(self):
        while not self.ctrl_c:
            self.pub.publish(self.move)
            self.rate.sleep()

# Main entry point of the program
if __name__ == "__main__":
    # Create an instance of the Explorer class
    explore = Explorer()
    try:
        # Run the main loop
        explore.main_loop()
    except rospy.ROSInterruptException:
        pass
