#!/usr/bin/env python

# This node provide a service for user to set a command in order 
# to choose robot behavior

# import ros stuff
import rospy
import time
from std_srvs.srv import *

## Global variable to know if the service is active
active_ = False


## Service callback function that modify value of active to activate or deactivate the service
# @param req User request, True to activate service, False to deactivate
# return res Return True to confirm that the opearation is done
def commandCallback(req):
    global active_ 
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

## Function to take and set user command 
def commandSetParam():
	global active_
	active_ = False
	
	cmd = 0
	
	time.sleep(0.5)
	
	print("\nPlease insert a new command:\n")
	print("1 - Achieve a randomly chosen point in the set following the move base algorithm\n")
	print("2 - Achieve a point choosen by user among those in the set following move base algorithm\n")
	print("3 - Start following external walls\n")
	print("4 - Stop in the last position\n")
	print("5 - change the planning algorith from move_base to bug0 or goes back to the original\n")
	print(" Pay attention that behavior 3,4,5 can be achieved only after having reached the previous target\n")
	
	while cmd < 1 or cmd > 5: 
		print("Press the number corrispondent to the desired behavior\n")
		cmd = int(raw_input('choosen command: '))
		if cmd != rospy.get_param("/user_command"):
			rospy.set_param("/user_command",cmd)
		else:
			rospy.set_param("/user_command",0)
		
		if cmd >= 1 and cmd <= 5:
			break
		else:
			print("Invalid command, please try again!\n")
			
	return []

## Main function to execute the service
def main():
	#initialize node
	rospy.init_node('user_command_server')
	
	global active
    
	#call request user command service
	srv = rospy.Service("/userCommand",SetBool,commandCallback)
	
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		if not active_:
			rate.sleep()
			
			# specific case 2 to avoid input collision and so when command is 2
			#wait until the target to get next command an leave time to think
			if rospy.get_param("/user_command") == 2:
				rospy.set_param("/user_command",0)
				#rospy.set_param("/target_reached", False)
				
			continue
		else:
			#set command in the ros param
			commandSetParam()
			time.sleep(1)

		rate.sleep()
        
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
	
