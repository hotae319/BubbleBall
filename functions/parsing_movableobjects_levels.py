'''
	@ Author : Hotae Lee
	@ Date : 05/25/2020
	@ Function : (1)Import json file and parse it while checking movable IDs and initial config (2) Enter inputs and run simulation automatically 
				--> parsing_objects(level_select) / run_simulation(level, movable_ID, ID_dict, state_input = [], logfileName = "bb"):	
	@ Parameters : 
	@ Variables:  level_select(int), state_input(list), logfileName(str)
	@ Retruns :  id_xxxx and s_xxxx (id list, state list) and n_total(list), ID_dict(dict), ID_state_matching(dict), movable_ID(list)
	@ Description : 
	@ TODO : Make a for loop to iterate many input values so that we can train this algorithm
'''

import json
import numpy as np
import subprocess
import time
import os, sys
'''
-----------------------------------------------------------------------------------
1) Import json files and parse all objects
-----------------------------------------------------------------------------------
'''
abspath = os.path.dirname(os.path.abspath(os.path.dirname(__file__)))
def parsing_objects(level_select):	

	'''
	Args:
	  level_select: Which level you choose
	Returns:
	  s_grd : ground's state (list)
	  s_total : other objects' initial state (list) 
	  id_total : objects ID (list)
	  n_total : the number of each type of objects (list)
	  movable_ID : list of movable ID (list)
	  ID_dict : The list of block type of each ID (dict)
	  ID_state_matching(dict) : ID to state
	'''
	# Open a level json file and movableObject list file
	
	with open('{0}/levels/{1}.json'.format(abspath,level_select), 'rt', encoding = 'utf-8-sig') as object_file:
		object_data = json.load(object_file)
	with open('{}/movableObjects.json'.format(abspath), 'rt', encoding = 'utf-8-sig') as movable_file:	
	    movable_data = json.load(movable_file) # Level object data

	# Change the ovable_data to a list (e.g. ['AO6G', 'C5NY'])
	movable_ID = movable_data["{}".format(level_select)]

	# Change the object_data to a list 
	# obj_list[k] means each object information ["type",x,y,width,height,rotation,"ID",""]
	obj_list = object_data[0]["objects"] 
	n_obj = len(obj_list)
	print("The total number of objects is {} in this level".format(n_obj))

	'''
	@ FYI
		The metal pieces and wood pieces can be distinguished by the state information.
		But, the types of speed buttons can be identified by only names, so we need to make each variable/list.
		The gravity and gravitydown also can be distinguished by only names. 
	'''

	# Initialize the information list of IDs and states
	id_grd = []
	id_metal = []
	id_wood = []
	id_woodtri = [] # circle and triagle has the same size 
	id_speedupu = []
	id_speedupd = []
	id_speedupl = []
	id_speedupr = []
	id_slowdown = []
	id_gravity = []
	id_gravitydown = []
	id_spring = []
	id_teleport = []

	s_grd = []
	s_metal = []
	s_wood = []
	s_woodtri = []
	s_speedupu = []
	s_speedupd = []
	s_speedupl = []
	s_speedupr = []
	s_slowdown = []
	s_gravity = []
	s_gravitydown = []
	s_spring = []
	s_teleport = []

	# Initialize the number of objects
	n_metal = 0 
	n_wood = 0
	n_woodtri = 0
	n_speedupu = 0
	n_speedupd = 0
	n_speedupl = 0
	n_speedupr = 0
	n_slowdown = 0
	n_gravity = 0
	n_gravitydown = 0
	n_spring = 0
	n_teleport = 0

	# Create ID_dict to check ID correspondency (e.g. Enter 'AOGC'-> show it is metal)
	ID_dict = {}
	ID_state_matching = {}
	# Classify all things according to types and Allocate all data to the lists
	for k in range(n_obj):
		a = obj_list[k]
		ID_state_matching['{}'.format(a[6])] = a[1:6]
		if a[0] == "ground":
			s_grd.append(a[1:6]) # [x,y,width,height,rotation]
			id_grd.append(a[6])			
		elif a[0] == "ball":
			s_ball = a[1:6]
			id_ball = a[6]
		elif a[0] == "flag":
			s_flag = a[1:6]
			id_flag = a[6]
		elif a[0][0:5] == "metal": # includes all metal blocks (name : metalzzzzz##x##)
			n_metal = n_metal + 1
			s_metal.append(a[1:6]) # this state's size element also represents the type of block(triangle, circle, rectangle)
			id_metal.append(a[6])
			ID_dict['{}'.format(a[6])] = 'metal block on {}'.format(a[1:6]) # Insert this into ID_dict
		elif a[0][0:4] == "wood" or a[0][0:4] =="catapult": # catapult has 125x25 size, so we can distinguish
			if a[0][0:7] == "woodrtr":
				n_woodtri += 1
				s_woodtri.append(a[1:6])
				id_woodtri.append(a[6])
				ID_dict['{}'.format(a[6])] = 'wood triangle on {}'.format(a[1:6])
			else:
				n_wood = n_wood + 1
				s_wood.append(a[1:6])
				id_wood.append(a[6])
				ID_dict['{}'.format(a[6])] = 'wood block on {}'.format(a[1:6])
		elif a[0] == "speedupu":
			n_speedupu = n_speedupu + 1
			s_speedupu.append(a[1:6])
			id_speedupu.append(a[6])
			ID_dict['{}'.format(a[6])] = 'speedupu'
		elif a[0] == "speedupd":
			n_speedupd = n_speedupd + 1
			s_speedupd.append(a[1:6])
			id_speedupd.append(a[6])
			ID_dict['{}'.format(a[6])] = 'speedupd'
		elif a[0] == "speedupl":
			n_speedupl = n_speedupl + 1
			s_speedupl.append(a[1:6])
			id_speedupl.append(a[6])
			ID_dict['{}'.format(a[6])] = 'speedupl'
		elif a[0] == "speedupr":
			n_speedupr = n_speedupr + 1
			s_speedupr.append(a[1:6])
			id_speedupr.append(a[6])
			ID_dict['{}'.format(a[6])] = 'speedupr'
		elif a[0] == "slowdown":
			n_slowdown = n_slowdown + 1
			s_slowdown.append(a[1:6])
			id_slowdown.append(a[6])
			ID_dict['{}'.format(a[6])] = 'slowdown'
		elif a[0] == "gravity":
			n_gravity = n_gravity + 1
			s_gravity.append(a[1:6])
			id_gravity.append(a[6])
			ID_dict['{}'.format(a[6])] = 'gravity'
		elif a[0] == "gravitydown":
			n_gravitydown = n_gravitydown + 1
			s_gravitydown.append(a[1:6])
			id_gravitydown.append(a[6])
			ID_dict['{}'.format(a[6])] = 'gravitydown'
		elif a[0] == "spring":
			n_spring = n_spring + 1
			s_spring.append(a[1:6])
			id_spring.append(a[6])
			ID_dict['{}'.format(a[6])] = 'spring'
		elif a[0][:8] == "teleport":
			n_teleport = n_teleport + 1
			s_teleport.append(a[1:6])
			id_teleport.append(a[6])
			ID_dict['{}'.format(a[6])] = 'teleport'
		else:
			print("new type of item appeared")

	# Make a list containing the number of each object
	id_total = [id_ball, id_flag, id_metal, id_wood, id_woodtri, id_speedupu, id_speedupd, id_speedupl, id_speedupr, id_slowdown, id_gravity, id_gravitydown, id_spring, id_teleport]
	s_total = [s_ball, s_flag, s_metal, s_wood, s_woodtri, s_speedupu, s_speedupd, s_speedupl, s_speedupr, s_slowdown, s_gravity, s_gravitydown, s_spring, s_teleport]
	n_total = [1, 1, n_metal, n_wood, n_woodtri, n_speedupu, n_speedupd, n_speedupl, n_speedupr, n_slowdown, n_gravity, n_gravitydown, n_spring, n_teleport]
	print("ID_dict shows that {}".format(ID_dict))
	print("The number of [ball, flag, metal, wood, woodtri, speedup (u,d,l,r), slowdown, grvity (u,d), spring, teleport] is {} ".format(n_total))
	print("IDs of metal and wood and woodtri are {0} and {1} and {2}".format(id_metal, id_wood, id_woodtri))
	print("The ID of the ball is {}".format(id_ball))
	print("The starting point of the ball is {0} and the target point of the flag is {1}".format(s_ball, s_flag))

	# Check which objects are movable to input initial states (from movableObjects.json)
	print("The movalbe IDs are {}\n".format(movable_ID))
	return id_grd, s_grd, s_total, id_total, n_total, movable_ID, ID_dict, ID_state_matching

'''
-----------------------------------------------------------------------------------
2) Enter inputs and run simulation automatically 
	- We can enter input values to the s[0],s[1],s[4] (x,y,r)
	- We can enter input values to the s[0],s[1] only (x,y) for powerup buttons (They have a fixed rotation)
	- e.g. { "unique": "AO6G", "x": 210, "y": 235, "r": 0 }
-----------------------------------------------------------------------------------
'''

def run_simulation(level, movable_ID, ID_dict, state_input = [], logfileName = "bb"):	
	'''
	Args:
	  level : Which level you choose
	  movable_ID : list of movable ID (list)
	  ID_dict : The list of block type of each ID (dict)
	  state_input : which state input you wanna enter
	  logfileName : Name of **.log
	Returns:
	  No return, just execute     
	'''
	# Import automate.json file to enter the desired input values
	with open('{}/automate.json'.format(abspath), 'rt', encoding = 'utf-8-sig') as automate_file:
		input_object = json.load(automate_file)
	obj_init = input_object["objectSetup"][0] # obj_init : dict, input_object["objectSetup"] : list
	print("The previous obj_init is {}".format(obj_init))

	# Enter the level and logfile name
	input_object["classicLevelNumber"] = level
	input_object["logFile"] = "{}.log".format(logfileName)

	# Clear everything on input_object["objectSetup"] list
	input_object["objectSetup"].clear()

	if state_input == []:		
		# 2)-1. Automatically input what you want on the terminal		
		for obj in movable_ID:
			state_input = input("Input the configuration [x,y,r] list of the {0} with \"{1}\" ID. You should enter the numbers by space: ".format(ID_dict['{}'.format(obj)], obj))
			state_input = state_input.split()
			input_object["objectSetup"].append({ "unique": "{0}".format(obj), "x": int(state_input[0]), "y": int(state_input[1]), "r": int(state_input[2])})
	else:
		# @ TODO : Make a for loop to iterate many input values so that we can train this algorithm
		pass
		# 2)-2. Manually input on the script
		# input_object["objectSetup"].append({ "unique": "AO6G", "x": 210, "y": 235, "r": 0 })
		# input_object["objectSetup"].append({ "unique": "AO6G", "x": 210, "y": 50, "r": 0 })

	# Update the automate.json file with what we wanna enter as an initial configuration
	with open('automate.json', 'w', encoding = 'utf-8-sig') as automate_file:
		json.dump(input_object, automate_file, indent = 2)
	with open('automate.json', 'rt', encoding = 'utf-8-sig') as automate_file:
		input_object = json.load(automate_file)	
	obj_init = input_object["objectSetup"][0]
	print("The changed obj_init is {}".format(obj_init))

	# Exectue finally
	#subprocess.run("./bubble-ball")
	#subprocess.run(["ls", "-l"])
	a = subprocess.Popen("./bubble-ball")
	time.sleep(5)
	a.terminate()
	gg = input("Check for python works after the bubble-ball execution")


if __name__ == "__main__":
	level_select = 9
	id_grd, s_grd, s_total, id_total, n_total, movable_ID, ID_dict, ID_state_matching = parsing_objects(level_select)
	print(ID_state_matching)
	print("s_grd : {}".format(s_grd))
	run_simulation(level_select, movable_ID, ID_dict)	
	