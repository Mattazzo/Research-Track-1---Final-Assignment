#!/usr/bin/env python

# This node implement Bug0 algorithm with addition of two service to get 
# a random target or to take a target from user input. There is also a timer 
# to avoid trying to reach impossible input, if timer expires the robot is 
# stopped and ask to user for a new command.

import rospy
import time
# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
# import ros service
from std_srvs.srv import *
from geometry_msgs.msg import Twist

import math

## Publisher for robot velocity
pub = None

## Client of go to point service
srv_client_go_to_point_ = None

## Client of wall follower service
srv_client_wall_follower_ = None

## Client of random target service
rdm_target_client = None

## Client of user target service
user_target_client = None

## Client of user command service
user_cmd_client = None

## Orientation angle
yaw_ = 0

## Orientation error allowed
yaw_error_allowed_ = 5 * (math.pi / 180)  # 5 degrees

## Robot position
position_ = Point()

## Target position
desired_position_ = Point()

## Iniitialization of target x coordinate
desired_position_.x = rospy.get_param('/target_x')

## Iniitialization of target y coordinate
desired_position_.y = rospy.get_param('/target_y')

## Iniitialization of target z coordinate
desired_position_.z = 0

## Regions of Laser callback
regions_ = None

## Possible states
state_desc_ = ['Go to point', 'wall following', 'target reached']

## Initial state
state_ = 2

## Service active or not ( fasle by default)
active_ = False

## variable to know if it is just started
start = 0

## Start time to reaching the target 
start_time = rospy.Time()

## Timeout for timer
timeout = rospy.Time()
# 0 - go to point
# 1 - wall following


## Callback service function to activate or deactivate the service
# @param req True to activate the service, False to deactivate
# @return res True to confirm that the operation is done
def bug0Callback(req):
	global active_
	active_ = req.data
	res = SetBoolResponse()
	res.success = True
	res.message = 'Done!'
	return res


## Callback function to know robot position
def clbk_odom(msg):
    global position_, yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


## Calback function to know laser detection values
def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }


## Function to change state in during the algorithm(wall follow, go to point or target reached)
def change_state(state):
	global state_, state_desc_,start
	global srv_client_wall_follower_, srv_client_go_to_point_
	global rdm_target_client, user_target_client, user_cmd_client
	
	state_ = state
	#log = "state changed: %s" % state_desc_[state]
	#rospy.loginfo(log)
	
	if state_ == 0:
		resp = srv_client_go_to_point_(True)
		resp = srv_client_wall_follower_(False)
		print("Robot is going to the target\n")
		
	if state_ == 1:
		resp = srv_client_go_to_point_(False)
		resp = srv_client_wall_follower_(True)
		print(" Robot is following the wall\n")
		
	if state_ == 2:
		resp = srv_client_go_to_point_(False)
		resp = srv_client_wall_follower_(False)
		twist_msg = Twist()
		twist_msg.linear.x = 0
		twist_msg.angular.z = 0
		pub.publish(twist_msg)
		print("Target reached\n")
		#resp = srv_client_user_interface_()
		
		#ask to user if he wants change command(not at the initializing change state)
		if start == 1:
			user_cmd_client(True)
		elif start == 0:
			start = 1
			
		#set target reached
		rospy.set_param("/target_reached", True)
 
		#give time to user to insert the command
		while rospy.get_param("/user_command") == 5:
			continue
			
			time.sleep(0.5)
		
		# calling right service to get command
		if rospy.get_param("/user_command") == 1:
			user_cmd_client(False)
			rdm_target_client()
		elif rospy.get_param("/user_command") == 2:
			user_cmd_client(False)
			user_target_client()
			

## Function to normalize angle
def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


## Main function which implement the Bug0 algorithm slightly modified
def main():
	time.sleep(2)
	global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_, active_
	global srv_client_go_to_point_, srv_client_wall_follower_, srv_rdm_target, srv_user_target, pub
	global rdm_target_client, user_target_client, user_cmd_client, start_time, timeout
	
	rospy.init_node('bug0')

	sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
	sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	
	srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)
    
	srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower', SetBool)
    
    #srv_client_user_interface_ = rospy.ServiceProxy('/user_interface', Empty)
    
    #my services
	rdm_target_client = rospy.ServiceProxy('randomTarget',Empty)
	user_target_client = rospy.ServiceProxy('userTarget',Empty)
	user_cmd_client = rospy.ServiceProxy('userCommand',SetBool)
    
	srv = rospy.Service('bug0_service', SetBool, bug0Callback)

    # initialize as already in the target( originally was 0)
	change_state(2)

	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		if not active_:
			rate.sleep()
			continue
		else:

			if state_ == 0:
				
				err_pos = math.sqrt(pow(desired_position_.y - position_.y,
										2) + pow(desired_position_.x - position_.x, 2))
				
				#timer expired
				if rospy.Time.now() >= timeout:
						print("TIMER EXPIRED \n")
						rospy.set_param("/user_command",4)
						rospy.set_param("target_reached",True)
						change_state(2)
						
				if(err_pos < 0.3):
					change_state(2)

				elif regions_['front'] < 0.5:
					change_state(1)

			elif state_ == 1:
				desired_yaw = math.atan2(
					desired_position_.y - position_.y, desired_position_.x - position_.x)
				err_yaw = normalize_angle(desired_yaw - yaw_)
				err_pos = math.sqrt(pow(desired_position_.y - position_.y,
										2) + pow(desired_position_.x - position_.x, 2))
				
				#timer expired
				if rospy.Time.now() >= timeout:
						print("TIMER EXPIRED set target as: \n")
						rospy.set_param("/user_command",4)
						rospy.set_param("target_reached",True)
						change_state(2)

				if(err_pos < 0.3):
					change_state(2)
				if regions_['front'] > 1 and math.fabs(err_yaw) < 0.05:
					change_state(0)

			elif state_ == 2:
				
				#give time to user to insert the command
				while rospy.get_param("/user_command") !=  1 and rospy.get_param("/user_command") != 2:
					continue
					
				#necessary for docker scheduling reason 
				time.sleep(0.5)
				
				# calling the right service to get target
				if rospy.get_param("/user_command") == 1 and active_:
					user_cmd_client(False)
					rdm_target_client()
					user_cmd_client(True)
				elif rospy.get_param("/user_command") == 2 and active_:
					user_cmd_client(False)
					user_target_client()
				
				
				desired_position_.x = rospy.get_param('/target_x')
				desired_position_.y = rospy.get_param('/target_y')
				err_pos = math.sqrt(pow(desired_position_.y - position_.y, 2) + pow(desired_position_.x - position_.x, 2))
					
				if(err_pos > 0.35):
					#set timer for this target (1 min)
					start_time = rospy.Time.now()
					timeout = start_time + rospy.Duration(60)
					print("Set timer, if target is not achieved in 1 minute, the robot will be stopped\n")
					
					change_state(0)

			rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
