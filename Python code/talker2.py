#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16MultiArray

def talker2():

	rospy.init_node('talker2', anonymous=True)    
	pub = rospy.Publisher('motor_control', Int16MultiArray, queue_size=10)

	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		data = Int16MultiArray()
		data.data = [0, 0]   #% rospy.get_time()
		#pub.data[1] = 100
		rospy.loginfo(data)
		pub.publish(data)
		rate.sleep()

if __name__ == '__main__':
    try:
        talker2()
    except rospy.ROSInterruptException:
        pass
