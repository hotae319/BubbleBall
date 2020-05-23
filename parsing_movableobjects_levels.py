import json
import numpy as np
import subprocess

level_select = 2

# Parsing code for objects' information depending on the level

with open('{}.json'.format(level_select), 'rt', encoding = 'utf-8-sig') as object_file:
	object_data = json.load(object_file)
with open('movableObjects.json', 'rt', encoding = 'utf-8-sig') as movable_file:	
    movable_data = json.load(movable_file) # Level object data

print(type(movable_data))
movable_ID = movable_data["{}".format(level_select)]
print(movable_ID)
print(type(movable_ID[0]))

# Parsing 
obj_list = object_data[0]["objects"] 
print(obj_list[0]) # obj_list[k] means each object information ["type",x,y,width,height,rotation,"ID",""]
n_obj = len(obj_list)
print("the total number of objects is {} in this level".format(n_obj))

'''
The metal pieces and wood pieces can be distinguished by the state information.
But, the speed type of buttons can be identified by only names, so we need to make each variables/lists.
gravity and gravitydown also can be distinguished by only names. 
'''

# Initialize the information list of ID and state
id_metal = []
id_wood = []
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
s_speedupu = []
s_speedupd = []
s_speedupl = []
s_speedupr = []
s_slowdown = []
s_gravity = []
s_gravitydown = []
s_spring = []
s_teleport = []

# Number of objects
n_metal = 0 
n_wood = 0
n_speedupu = 0
n_speedupd = 0
n_speedupl = 0
n_speedupr = 0
n_slowdown = 0
n_gravity = 0
n_gravitydown = 0
n_spring = 0
n_teleport = 0

# ID dictionary
ID_dict = {}
for k in range(n_obj):
	a = obj_list[k]
	if a[0] == "ground":
		s_grd.append(a[1:6]) # [x,y,width,height,rotation]
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
		ID_dict['{}'.format(a[6])] = 'metal'
	elif a[0][0:4] == "wood" or "catapult": # catapult has 125x25 size, so we can distinguish
		n_wood = n_wood + 1
		s_wood.append(a[1:6])
		id_wood.append(a[6])
		ID_dict['{}'.format(a[6])] = 'wood'
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

n_total = [n_metal, n_wood, n_speedupu, n_speedupd, n_speedupl, n_speedupr, n_slowdown, n_gravity, n_gravitydown, n_spring, n_teleport]
print(ID_dict)
print("The number of [metal, wood, speedup (u,d,l,r), slowdown, grvity (u,d), spring, teleport] is {} ".format(n_total))
print("IDs of metal and wood are {0} and {1}".format(id_metal, id_wood))
print("Initial states of metal is {}".format(s_metal))
print("The ID of the ball is {}".format(id_ball))
print("The starting point is {0} and the target point is {1}".format(s_ball, s_flag))

# When we input ID, we can check what that ID is


# Check which objects are movable to input initial states (from movableObjects.json)
# We can allocate values to the s[0],s[1],s[4] (x,y,r)
# We can allocate values to the s[0],s[1] only (x,y) for powerup buttons
#  { "unique": "AO6G", "x": 210, "y": 235, "r": 0 }

print("movalbe ID is {}".format(movable_ID))

with open('automate.json', 'rt', encoding = 'utf-8-sig') as automate_file:
	input_object = json.load(automate_file)
obj_init = input_object["objectSetup"][0] # it became dictionary after removing the list

print("The original obj_init is {}".format(obj_init))

# Input what you want 
input_object["objectSetup"].clear()
# Automatically input on the terminal
for obj in movable_ID:
	state_input = input("Input the configuration [x,y,r] list of the {0} object with {1} ID. You should enter the numbers by space: ".format(ID_dict['{}'.format(obj)], obj))
	state_input = state_input.split()
	input_object["objectSetup"].append({ "unique": "{0}".format(obj), "x": int(state_input[0]), "y": int(state_input[1]), "r": int(state_input[2])})
# Manually input on the script
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
subprocess.run("./bubble-ball")
#subprocess.run(["ls", "-l"])

'''
Output list we can use : id_xxxx and s_xxxx (id list, state list) and n_xxxx
'''