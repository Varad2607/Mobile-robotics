#!/usr/bin/env python

import rospy
import rosbag
from ackermann_msgs.msg import AckermannDriveStamped

BAG_TOPIC = '/car/mux/ackermann_cmd_mux/input/teleop'
PUB_TOPIC = '/car/mux/ackermann_cmd_mux/input/teleop'

def follow_bag(bag_path, follow_backwards=False):

	pub = rospy.Publisher('/car/mux/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=10)
	bag = rosbag.Bag(bag_path)
	rate = rospy.Rate(10)
	messages = list(bag.read_messages(topics=[BAG_TOPIC]))
	if follow_backwards == True:
		messages.reverse()
		start_time = rospy.Time.now()
		for topic, msg, t in messages:
			if not rospy.is_shutdown():
				if follow_backwards:
					elapsed_time = start_time - msg.header.stamp
					msg.header.stamp = rospy.Time.now() - elapsed_time
					msg.drive.steering_angle *= -1
					msg.drive.speed *= -1
				else:
					msg.header.stamp = rospy.Time.now()
				
				pub.publish(msg)
				rate.sleep()

	else:
		for topic, msg, t in bag.read_messages(topics=[BAG_TOPIC]):
			if rospy.is_shutdown():
				break
			pub.publish(msg)
			rate.sleep()		

	bag.close()


if __name__ == '__main__':

	rospy.init_node("bag_follower", anonymous=True)
	bag_path = rospy.get_param('~bag_path', '/home/robot/mushr_ws/src/mushr545/assignment2/rosbags/simulation_figure_8_rosbag.bag')
	follow_backwards = rospy.get_param('~follow_backwards', True)
	follow_bag(bag_path, follow_backwards)
