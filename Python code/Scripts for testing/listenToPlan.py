#!/usr/bin/env python


import rospy
from std_msgs.msg import String

global plan_list

plan_list = []

def plan_callback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    # Saves the data in its string form
	info_gotten = data.data
	# Splits the data string and stores them in a list
	splitted_data = info_gotten.split(' ')
	if splitted_data[0] == "INITIAL_PART_LIST":
		parts = splitted_data
		# Removes the message 'header' (INITIAL_PART_LIST) from the list
		parts.pop()
		# Removes a redundant entry in the end of the list
		parts.pop(0)
		# Prints all the parts in the list
		for part in parts:
			print(part)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('plan_whisperer', String, plan_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



if __name__ == '__main__':
    listener()
