#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from products import Product

product_list = []
""" Takes in the recieved data and converts it to objects of class Product"""
def plan_callback(data):
	global product_list
	# Clears the product list to ensure no duplicates
	product_list = []
    # Saves the data in its string form
	string_to_split = data.data
	# Splits the string into lines by separating with each newline character
	data_lines = string_to_split.split("\n")
	# Goes through the lines and splits them by the tab character
	# Then uses the split values to create objects of type Product
	for line in data_lines:
		temp_id, temp_type, temp_status = line.split("\t")
		product_list.append(Product(temp_id, temp_type, status=temp_status))
	# Initializes a string for logging
	string_to_log = ""
	# Goes through the products and adds their info to the log string
	for product in product_list:
		string_to_log += (str(product.id) + "\t" + product.type + "\t" + product.status + "\n")
	# Strips the log string of its last character (\n)
	string_to_log = string_to_log[:-1]
	# Uses rospy to log the log string
	rospy.loginfo("Received plan:\n" + string_to_log)


def listener():
	# Initializes this node as the robot_planner
	rospy.init_node('robot_planner', anonymous=True)
	# Subscribes to messages of type String from interface_plan_talker
	rospy.Subscriber('interface_plan_talker', String, plan_callback)
	# spin() keeps python from exiting until this node is stopped
	rospy.spin()

# If this file is running as the main file call the listener function
if __name__ == "__main__":
	listener()
