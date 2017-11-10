#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from products import Product

product_list = []
""" """
def plan_callback(data):
	global product_list
	# Clears the product list to ensure no duplicates
	product_list = []
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    # Saves the data in its string form
	string_to_split = data.data
	
	data_lines = string_to_split.split("\n")

	for line in data_lines:
		temp_id, temp_type, temp_status = line.split("\t")
		product_list.append(Product(temp_id, temp_type, status=temp_status))

	string_to_log = ""
	for product in product_list:
		string_to_log += (str(product.id) + "\t" + product.type + "\t" + product.status + "\n")
	string_to_log = string_to_log[:-1]
	rospy.loginfo("Received plan:\n" + string_to_log)


def listener():
    rospy.init_node('robot_planner', anonymous=True)

    rospy.Subscriber('interface_plan_talker', String, plan_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
	listener()
