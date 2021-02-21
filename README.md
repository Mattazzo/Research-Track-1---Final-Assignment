# Research Track I - final assignment
Matteo Azzini 4475165

The assignment requires to develop a software architecture for the control of the robot in the environment. 
The software will rely on the move_base and gmapping packages for localizing the robot and plan the motion.

The architecture should be able to get the user request, and let the robot execute one of the following behaviors (depending on the user's input):

	1 - Move randomly in the environment, by choosing 1 out of 6 possible target positions: [(-4,-3);(-4,2);(-4,7);(5,-7);(5,-3);(5,1)]
	
	2 - Directly ask the user for the next target position (checking that the position is one of the possible six) and reach it
	
	3 -	Start following the external walls
	 
	4 - Stop in the last position
	 
	5 - Change the planning algorithm from dijkstra (move_base) to the bug0 and viceversa
	
	
If the robot is in state 1, or 2, the system should wait until the robot reaches the position in order to switch to the state 3 and 4 or to change the planning algorithm.


### Content of the package
- **docs**   		: It's a folder containing online and offline documentation about this package 
- **scripts**		: It's a folder containing all Python executable used in this package 
- **worlds** 		: It's a folder that define the world within the robot should move
- **launch**		: It's a folder containing all file with .launch extension needed to launch the simulation
- **urdf**			: It's a folder containing all information and description about the robot 
- **param**			: It's a folder containing all file with .yaml extension neede to set parameters for the simulation
- **config**		: It's a folder containing a file useful for simulation tools
- **rosgraph.png**	: It's an image which graphically represent how nodes comuinicate and which topics they use
- **CMakeList.txt** : It's a file with informations about the compilation 
- **package.xml**	: It's a file with informations about the compilation

In particular the folder 'scripts' contain:
	- robot_controller.py 		: Principal node that control robot behaviour 
	- user_command_server.py 	: Implement the server to take user command
	- random_target_server.py 	: Implement the server to set a random target for the robot
	- user_target_server.py 	: Implement the server to set a target for the robot taken from user input
	- bug_m.py					: Implement a server to execute bug0 algorithm(professor version slightly modified)
	- wall_follow_service_m.py	: Implement a server to let robot follow the wall, used also in bug0 algorithm (professor version)
	- go_to_point_service_m.py	: Implement a server to let robot go to a desired point, used also in bug0 algorithm (professor version)

	
### How to run the simulation
In order to run the simulation there is a launch file that can be launched with the followig command:

```
roslaunch final_assignment final_assignment.launch 
```

It includes launch file for Gazebo and Rviz graphic user interfaces, launch file for move base algorithm and all necessary parameters and nodes to execute the simulation.


### Robot behaviours
When the simulation is launched the robot is spawned in the pre-built environment waiting for a command from user, once the command is given the robot can perform 5 different behaviour:
	
	1 - A random target is given from the random_target_server and the robot try to reach it following the active algorithm (by default is move base).
		In the maintime the user can insert the next command to be execute, once the robot has achieved the target, informs user printing on terminal and execute the next command.
		
	2 - The robot requires a target by user thanks to user_target_server, once user has inserted it the robot try to reach it with the active algorithm.
		When the robot reach the target, it informs user and hask him for a new command.
		
	3 - Robot starts following the wall, in the maintime the user can insert the next command to be executed.
	
	4 - Robot is stopped and asks user for the next command.
	
	5 - Moving algorithm is switched and robot asks for the next command. 
	
Behavior 3,4,5 can be executed only when robot has reached the target in 1 or 2, not during the execution.
	
Attention: move_base has a recovery behavior for unreachable target, bug0 doesn't, so I have implemented a timeout to avoid robot tryng to reach unfiesable targets, when the timer expires the robot stop itself and ask for a new command from user.

	
### About software architecture
I decided to have 4 parameters (target coordinates x and y, command inserted by user, a flag to check if the target is reached) in order to allow all nodes to check and modify them.
When move base algorithm is active all behaviors are managed by the node robot_controller.py, which calls user_commmand_server.py when it's necessary to get a command.
Once the command is inserted by the user, the node activates the corrispondent service to execute the desidered behavior, so it interacts with all services.
Instead when bug0 is active the node robot_controller.py is always responsable to call service to get user command and execute command 3,4,5 but for behavior 1 and 2 is the node bug_m.py that call services to get a target (randomly or from user).
bug_m.py is also responsable to call wall_follow_service.py and go_to_point_service.py to execute the bug0 algorithm to achieve the target and it is its duty also to set the timer for the recovery behavior (when robot try to reach unfeasible target) and to stop the robot wen the timer expires.

For a graphic representation of the software achitecture, please check final_assignment_rosgraph.png.


### System limitations and possible improvements
**Limitations**:
	- Sometimes the map can change position during robot motion, this obviously creates problem for robot localization 
	- Sometimes the input is not detected, so when you press a number and it he acquires a null character
	- Sometimes it seems that Docker change scheduling order of my scripts unreasonably (so in very rarely cases input for commands overlaps input for target)
	
I think that these are problems related to Docker execution on my PC, but I cannot be sure of this.

**Possible improvements**:
	- There is a print about laser scanner, probably due to move base nodes, I would delete it to have a cleaner output on the terminal.
	- Find a more efficent recovery behavior for bug0 instead of the timer.
	- Print robot position or distance from the target when robot is moving. 
	
