#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Minimum distance to obstacles (in meters)
MIN_DISTANCE = 0.5

def callback(data):

    # Get the range data from the Lidar
    ranges = data.ranges


    # Check the front, left, right, and rear ranges
    front_distance = min(ranges[160:200])  # 40-degree arc in front
    left_distance = min(ranges[225:315])  # 90-degree arc to the left
    right_distance = min(ranges[45:135])  # 90-degree arc to the right

    # Handle the rear range check differently to account for the 360 degree wrap-around
    rear_ranges = ranges[315:] + ranges[:45]  # Combine the two halves of the rear arc
    rear_distance = min(rear_ranges)
    print(front_distance)

    # Determine the appropriate action based on the obstacle distances
    if front_distance < MIN_DISTANCE:
        # Obstacle detected, stop the robot and turn
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.6
        pub.publish(twist)
    elif front_distance > MIN_DISTANCE:
        twist = Twist()
        twist.linear.x = -0.4
        twist.angular.z = 0.0
        pub.publish(twist)
    elif left_distance < MIN_DISTANCE:
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = 0.6
        pub.publish(twist)
    elif right_distance < MIN_DISTANCE:
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = -0.6
        pub.publish(twist)

    else:
        # No obstacles, move forward
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = 0.0
        pub.publish(twist)
        print("linear published")

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('obstacle_avoidance')
    rate = rospy.Rate(10) #10Hz


    # Subscribe to the Lidar topic
    sub = rospy.Subscriber('/scan', LaserScan, callback)

    # Create a publisher for the velocity commands
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate.sleep()

    # Keep the node running
    rospy.spin()
