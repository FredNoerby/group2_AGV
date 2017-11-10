#!/usr/bin/env python

import rospy #Import python lib
from geometry_msgs.msg import TransformStamped #Import the specefik massage type
from std_msgs.msg import Empty


def Codicomand():#Defining the fuction
	rospy.init_node('Robot_Comand', anonymous=True) #Defining this to be a node and its name as the masternode knows it
	pup_com = rospy.Publisher('/Move_Cordinats', TransformStamped, queue_size=10) #Defining and saving a publisher function, and naming whats the name of the topic and what massage type it uses 	
	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
        
		
		#Move_info = Empty()
		Move_info = TransformStamped() #Save a massage with the TransforStamped as the type

		#Now we set all of its vectors values to 0
		Move_info.transform.translation.x = 2 
		Move_info.transform.translation.y = 2
		Move_info.transform.translation.z = 3

        	rospy.loginfo(Move_info)
		pup_com.publish(Move_info) #Exicute the puplish comand with the Move_info as the topic.

        	rate.sleep()



if __name__ == '__main__': #Saying if any other fu
	 
    try:
        Codicomand()
    except rospy.ROSInterruptException:
        pass

