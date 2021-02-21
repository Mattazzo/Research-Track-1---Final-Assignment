#!/usr/bin/env python

# This node implement the robot behavior looking at the command insert by 
# the user, each command correspond to a certain behavior.
# By defaul robot follow Move Base alghoritm, it can achieve a random target
# or a target specified by user, it can also start to follow the external 
# walls or can be stopped in the last position.
# Finally the node allows to change the algoritm to Bug0 and come back to 
# achieve a random or chosen by user target.   

# import ros stuff
import rospy
import time
from std_srvs.srv import *
from move_base_msgs.msg import MoveBaseActionGoal,MoveBaseActionResult
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID


## Publisher for move_base goal
MB_target_pub = rospy.Publisher("move_base/goal", MoveBaseActionGoal, queue_size=1)

## Publisher to cancel a goal for move base
MB_cancel_target_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size = 1)

## Publisher to stop the robot
pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

## variable to know if robot is stopped (stop = 1)
stop = 0

## Function to set a goal for Move Base algorithm
def setMoveBaseGoal():
	global MB_target_pub
	
	#set the goal for move_base alg
	move_base_goal = MoveBaseActionGoal()
	
	move_base_goal.goal.target_pose.header.frame_id = "map"
	move_base_goal.goal.target_pose.pose.orientation.w = 1
	move_base_goal.goal.target_pose.pose.position.x =  rospy.get_param("/target_x")
	move_base_goal.goal.target_pose.pose.position.y =  rospy.get_param("/target_y")
			
	#publish the msg
	MB_target_pub.publish(move_base_goal)
	
	
## Callback function of subscriber to chek if a terget is reached	
def targetReachedCallback(result):
	global stop
	
	reached = 0
	
	status = result.status.status
	
	if status == result.status.SUCCEEDED:
		rospy.set_param("/target_reached",True)
		if reached == 0 and stop == 0:
			reached = 1
			print("Target reached!\n")
			
	else:
		rospy.set_param("/target_reached",False)
		reached = 0
	

## Main function to interact with user and chek user command updating robot behavior
def main():
	time.sleep(3) #to allow bug0 start before 
	rospy.init_node('robot_controller')
    
	#var to know if target is reached and I can change behavior
	global pub_vel,stop#,target_reached 
	#target_reached = True
	rospy.set_param("/target_reached",True)
	
	# 1 if I'm using move base, 0 if I'm using bug0
	mb_bug0 = 1
	
	#used fro don't repeat print in stopped case
	stop = 0
	
	# 1 if robot is following wall
	wall_follow = 0
	
	# variable to make a command request in state 0 
	make_req = 0
	
	# variable use to avoid multiple change of algorithm
	switched = 0
	
	#services
	user_cmd_client = rospy.ServiceProxy('/userCommand',SetBool)			#get user command
	rdm_target_client = rospy.ServiceProxy('/randomTarget',Empty)			#get a random target
	user_target_client = rospy.ServiceProxy('/userTarget',Empty)			#get a target from user
	wall_follow_client = rospy.ServiceProxy('/wall_follower',SetBool)		#follow walls
	go_to_point_client = rospy.ServiceProxy('/go_to_point_switch',SetBool) 	#go to point service
	bug0_client = rospy.ServiceProxy('/bug0_service',SetBool)				#bug0 algorithm
	
	#chek if move_base has reached the target
	rospy.Subscriber("/move_base/result", MoveBaseActionResult, targetReachedCallback)

	#starting print to inform user
	print("Hi! This robot, given a point set = [(-4,-3);(-4,2);(-4,7);(5,-7);(5,-3);(5,1)], can perform 5 behaviors in a pre-built environment:\n")
    
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		
		#ask user for a command
		user_command = rospy.get_param("/user_command")
		
		#correspondent behaviors
		if user_command == 1 and rospy.get_param("/target_reached") == True: 
			#target_reached = False
			rospy.set_param("/target_reached",False)
			
			#stop other services
			wall_follow_client(False)
			user_cmd_client(False)
			go_to_point_client(False)
			
			#set needed variables
			wall_follow = 0
			stop = 0
			make_req = 0
			switched = 0
			
			#if Move Base si active
			if mb_bug0 == 1:
				#get random target
				rdm_target_client()
			
				# set target for Move Base algorithm
				setMoveBaseGoal()
				
				#renable service to get command ( suspended to avoid collision)
				user_cmd_client(True)
			
			#if Bug0 is active
			elif mb_bug0 == 0:
				make_req = 0
				bug0_client(True)
				user_cmd_client(True)
				
		
		elif user_command == 2 and rospy.get_param("/target_reached") == True: 
			
			rospy.set_param("/target_reached",False)
			
			#stop other services
			wall_follow_client(False)
			user_cmd_client(False)
			go_to_point_client(False)
			
			#set needed variables
			wall_follow = 0
			stop = 0
			make_req = 0
			switched = 0
			
			# if Move Base is active
			if mb_bug0 == 1:
			
				#ask user for a target
				user_target_client() 
			
				# set target for Move Base algorithm
				setMoveBaseGoal()
			
			# if Bug0 is active
			elif mb_bug0 == 0:
				make_req = 0
				bug0_client(True)
			
			
		elif user_command == 3 and rospy.get_param("/target_reached") == True:
				
			# service can be activated
			if wall_follow == 0:
				print("start following wall\n")
				
				# set needed variables
				wall_follow = 1
				make_req = 0
				stop = 0
				switched = 0
				
				#stop other services
				bug0_client(False)
				go_to_point_client(False)
				
				#start wall follow service
				wall_follow_client(True)
					
				#renable ser ice to get command ( suspended to avoid collision)
				user_cmd_client(True)
				
			
		elif user_command == 4 and rospy.get_param("/target_reached") == True: 
			
			# robot can be stopped
			if stop == 0:
				print("Robot is stopped\n")
				stop = 1
				make_req = 0
				wall_follow = 0
				switched = 0
				
				#stop other services
				user_cmd_client(False)
				wall_follow_client(False)
				bug0_client(False)
				go_to_point_client(False)
				
				# set zero velocity
				vel = Twist()
				vel.linear.x = 0
				vel.linear.y = 0
				vel.angular.z = 0
				pub_vel.publish(vel)
				
				# ask user command
				user_cmd_client(True) 
				
		elif user_command == 5 and rospy.get_param("/target_reached") == True:  
			
			#rospy.set_param("/target_reached",False)
			
			# stop other services
			user_cmd_client(False)
			#wall_follow_client(False)
			
			# set needed variables
			wall_follow = 0
			stop = 0
			
			# if Bug0 is not active can be activated
			if mb_bug0 == 0 and switched == 0:
				mb_bug0 = 1
				switched = 1
				bug0_client(False)
				rospy.set_param("/target_reached",True)
				rospy.set_param("user_command",0)
				print("Activated Move Base algorithm\n")

			# Bug0 is active can be deactivated
			elif mb_bug0 == 1 and switched == 0:
				mb_bug0 = 0
				switched = 1
				bug0_client(True)
				print("Activated bug0 algorithm\n")
			
			#renable user command service
			user_cmd_client(True)
		
		elif user_command == 0 and rospy.get_param("/target_reached") == True: 
			if make_req == 0:
				user_cmd_client(True)
				make_req = 1
		
		rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
	
