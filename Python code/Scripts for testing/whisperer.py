#!/usr/bin/env python

import rospy
from random import randint
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('assembly_line', String, queue_size=10)
    rospy.init_node('assembly_talker', anonymous=True)
    rate = rospy.Rate(0.3) # 10hz
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        in_assembly_str = (str(randint(0, 6)) + " " + str(randint(0, 6)) + " " + str(randint(0, 6)) + " " + str(randint(0, 6)) + " " + str(randint(0, 6)) + " " + str(randint(0, 6)))
        rospy.loginfo(in_assembly_str)
        pub.publish(in_assembly_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
