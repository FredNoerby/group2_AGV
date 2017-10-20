#CODE
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data) # publish it on screen and put data + I heard in loginfo
    
def listener():

	rospy.init_node('listener', anonymous=True) 	#if malfunctioning nodes appear it will kicked out
	rospy.Subscriber("chatter", String, callback)  	#declares that node is subscribe to the chatter,includes call back
													#declares to the chatter (node)

					# spin() simply keeps python from exiting until this node is stopped
	rospy.spin() 	#keeps the node from exiting until the node is shutdown

if __name__ == '__main__':
	listener()


#commands
#$ chmod +x listener.py


#cd ~/catkin_ws
#catkin_make
