#!/usr/bin/env python3

import rospy
import os
import signal
import rospkg

save_map = False

def sigint_handler(sig, frame):
    global save_map
    rospy.loginfo("Initialised node to save map")
    save_map = True

def main():
    global save_map
    rospack = rospkg.RosPack()

    rospy.init_node("map_saver_node")

    signal.signal(signal.SIGINT, sigint_handler)
    rospy.loginfo("Saving map...")

    while not rospy.is_shutdown():
        if save_map:
            map_name = "map/map"  # Modify this with your map name
            map_dir = rospack.get_path('acs6121_team1')  # Modify this with your desired directory
            map_file = os.path.join(map_dir, map_name)

            rospy.loginfo("Saving map to: %s", map_file)

            cmd = "rosrun map_server map_saver -f {}".format(map_file)
            os.system(cmd)

            rospy.loginfo("Map saved successfully.")

            save_map = False

        rospy.sleep(0.1)

if __name__ == "__main__":
    main()
