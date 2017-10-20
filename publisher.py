#commands
#mkdir scripts
#cd scripts
#rosed name.py
#chmod +x talker.py

#CODE

#!/usr/bin/env python 	# declaration of a python script
# license removed for brevity	

import rospy	#import library for the ROS
from std_msgs.msg import String 	# use of string container for publishing
	

def talker():  	#function

	
	pub = rospy.Publisher('chatter', String)	#safe topic 
 
	rospy.init_node('talker', anonymous=True) 	#tells the node name if ROS does not have this info it cannot start communicating with the Master. 
												#Use only base name. anonimous- Adds unique name 

	rate = rospy.Rate(10) # 10hz 	#create  object  rate. Looping at 10hz (10 times per second)

	while not rospy.is_shutdown(): 	#checking is it shut down if yes then exit if no then call next line
		hello_str = "hello world %s" % rospy.get_time() #it puts the value of time as string	 
		rospy.loginfo(hello_str)	#keep in log file, messages printed on the screen
		pub.publish(hello_str) 	# publish string in chatter
        rate.sleep()	#sleep 1/10 of a second
		 
if __name__ == '__main__': 	#if other python access the script and it will not be the main script
	try:
		talker()
	except rospy.ROSInterruptException: 	#exception for not continue executing the code sleep()
		pass		#end
