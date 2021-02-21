#!/usr/bin/env python

# This node implement a service to chose a target to be achieved by the robot

# import ros stuff
import rospy
from std_srvs.srv import *

##Callback function to take target from
#@param request Empty  
def targetCallback(request):
	while True:
		print("Choose one point among [(-4,-3);(-4,2);(-4,7);(5,-7);(5,-3);(5,1)]\n")
		x = float(raw_input('target x coordinate:'))
		y = float(raw_input('target y coordinate:'))
			
		#check if is an avaible target otherwise repeat 
		if x == -4:
			if y == -3 or y == 2 or y == 7:
				break
			else:
				print("Invalid target point, please try again\n")
		elif x == 5: 
			if y == -7 or y == -3 or y == 1:
				break
			else:
				print("Invalid target point, please try again\n")
		else:
			print("Invalid target point, please try again\n")
	
	rospy.set_param("/target_x",x)
	rospy.set_param("/target_y",y)
	
	print("Chosen target [" + str(rospy.get_param("/target_x")) + ";" + str(rospy.get_param("/target_y")) + "]\n") 
	
	return []

## Main function to execute the service
def main():
	#initilize node
	rospy.init_node('user_target_server')
 
	#call user target request service
	srv = rospy.Service("/userTarget",Empty,targetCallback)
	
	rospy.spin()
        
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
	
