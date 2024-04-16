#!/usr/bin/env python3
# A simple ROS publisher node in Python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import roslaunch
from std_srvs.srv import Empty
import rospkg
import time

# Importing required module
import subprocess

class explorer():
    def callback_function(self, dt):
        thr1 = 0.8 # Laser scan range threshold
        thr2 = 0.8
        self.move = Twist() # Creates a Twist message type object
        if dt.ranges[0]>thr1 and dt.ranges[15]>thr2 and dt.ranges[345]>thr2: # Checks if there are obstacles in front and
                                                                            # 15 degrees left and right (Try changing the
                                        # the angle values as well as the thresholds)
            self.move.linear.x = 0.1 # go forward (linear velocity)
            self.move.angular.z = 0.0 # do not rotate (angular velocity)
        else:
            self.move.linear.x = 0.0 # stop
            self.move.angular.z = 0.5 # rotate counter-clockwise
            if dt.ranges[0]>thr1 and dt.ranges[15]>thr2 and dt.ranges[345]>thr2:
                self.move.linear.x = 0.1
                self.move.angular.z = 0.0

    def __init__(self):
        self.move = Twist() # Creates a Twist message type object
        rospy.init_node('obstacle_avoidance_node') # Initializes a node
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  # Publisher object which will publish "Twist" type messages
                                                    # on the "/cmd_vel" Topic, "queue_size" is the size of the
                                                                # outgoing message queue used for asynchronous publishing

        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback_function)  # Subscriber object which will listen "LaserScan" type messages
                                                            # from the "/scan" Topic and call the "callback" function
                                    # each time it reads something from the Topic
        
        
        
        self.startup = True

        # This might be useful in the main_loop() (to switch between 
        # turning and moving forwards)
        self.turn = False
        self.rate = rospy.Rate(10)  # hz

        # define a Twist message instance, to set robot velocities
        self.vel = Twist()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self):
        # publish an empty twist message to stop the robot
        # (by default all velocities will be zero):
        self.pub.publish(Twist())
        self.ctrl_c = True
        # Save the map
        # self.save_map()

    def save_map(self):
        rospy.loginfo("Saving map...")
        # Initialize the ROS package manager
        rospack = rospkg.RosPack()

        # Get the path to the package
        package_path = rospack.get_path('acs6121_team1')

        # Define the path to the "map" folder inside your package
        map_folder_path = package_path + '/map/map1'
        # print(map_folder_path)
        try:
            time.sleep(5)

            subprocess.run('rosrun map_server map_saver -f '+map_folder_path, shell=True)
            subprocess.Popen('echo test', shell=True)

            rospy.logwarn("Map saved successfully to" + map_folder_path)
            time.sleep(5)

        except rospy.ServiceException as e:
            rospy.logerr("Failed to save map: %s" % e)


    def main_loop(self):
        while not self.ctrl_c:

            self.pub.publish(self.move) # publish the move object

            self.rate.sleep()


if __name__ == "__main__":
    explore = explorer()
    try:
        explore.main_loop()
    except rospy.ROSInterruptException:
        pass
