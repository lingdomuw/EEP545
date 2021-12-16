#!/usr/bin/env python

import rospy
import rosbag
from ackermann_msgs.msg import AckermannDriveStamped

BAG_TOPIC = '/car/mux/ackermann_cmd_mux/input/teleop' # Name of the topic that should be extracted from the bag
PUB_TOPIC = '/car/mux/ackermann_cmd_mux/input/teleop'
PUB_RATE = 10 # The rate at which messages should be published

# Loads a bag file, reads the msgs from the specified topic, and republishes them
def follow_bag(bag_path, follow_backwards):
	publisher = rospy.Publisher (PUB_TOPIC, AckermannDriveStamped, queue_size=10)
	path_bag = rosbag.Bag (bag_path)
	pub_rate = rospy.Rate (PUB_RATE)

	for topic, msg, t in path_bag.read_messages (topics=BAG_TOPIC):
		if follow_backwards:
			msg.drive.speed *= -1
		
		publisher.publish (msg)
		pub_rate.sleep ()

	path_bag.close ()



if __name__ == '__main__':
	bag_path = None # The file path to the bag file
	follow_backwards = False # Whether or not the path should be followed backwards
	
	rospy.init_node('bag_follower', anonymous=True)
	
	# Populate param(s) with value(s) passed by launch file
	bag_path = rospy.get_param ('bag_path')	
	follow_backwards=rospy.get_param('follow_backwards')

	fg=follow_bag(bag_path, follow_backwards)
	rospy.spin() # Spin
