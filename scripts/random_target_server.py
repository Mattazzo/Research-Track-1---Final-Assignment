#!/usr/bin/env python
 
# This node is the server which provide a random target for the robot 

import rospy
import random
from std_srvs.srv import *
	
## Server function to give a target for robot 
# @param request Empty
def randomTargetCallback(request):
	
	#Target for robot
	rdm = int(random.uniform(1,6))

	if rdm == 1:
		rospy.set_param("/target_x",-4)
		rospy.set_param("/target_y",-3)
	elif rdm == 2:
		rospy.set_param("/target_x",-4)
		rospy.set_param("/target_y",2)
	elif rdm == 3:
		rospy.set_param("/target_x",-4)
		rospy.set_param("/target_y",7)
	elif rdm == 4:
		rospy.set_param("/target_x",5)
		rospy.set_param("/target_y",-7)
	elif rdm == 5:
		rospy.set_param("/target_x",5)
		rospy.set_param("/target_y",-3)
	elif rdm == 6:
		rospy.set_param("/target_x",5)
		rospy.set_param("/target_y",1)
		
	print("The random target is [" + str(rospy.get_param("/target_x")) + ";" + str(rospy.get_param("/target_y")) + "]\n")
		
	return []
	
	
## Main function to execute the service 
def main():
	
	#node initialization
	rospy.init_node('random_target_server')
	
	#call  Target service
	srv = rospy.Service("/randomTarget",Empty,randomTargetCallback)
	
	rospy.spin()
	
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
	
   
