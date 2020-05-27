'''
	@ Author : Hotae Lee
	@ Date : 05/25/2020
	@ Function : (1)Import log file and classify the trajectories according to ID (2) Save when collisions occur and what collides 			
					--> logging_trajectory(filename)			
	@ Parameters : 
	@ Variables : 
	@ Retruns :  traj_list[k](np array), collision(list), trace_time(list), n_obj(int), bool_success(bool), ID_list(list)
	@ Description : 
	@ TODO : 
'''

import numpy as np
import parsing_movableobjects_levels as ps
'''
-----------------------------------------------------------------------------------
Import json files and parse all objects
-----------------------------------------------------------------------------------
'''


# Change the log file to the matrix/vector form

'''
From the log file, we have to get t-vector, state-vector [vx,vy,v_ang,x,y,rotation] of each object
Those objects would be only a ball or wood blocks. It's not necessary to get other things' trajectory since they are stationary. 
e.g. traj1 = N by 7 matrix = [time(1:N).T, state with 6 elements(1:N).T] (traj1 is for each ID)

Thus, after this code runs, we can have n_obj matrices like traj_list[k] corresponding to traj_bal, traj_block1, traj_block2,...
'''
def logging_trajectory(filename, level_select):		
	'''
	Args:
	  filename : **.log
	Returns:
	  traj_list :  The list of each object's trajectory (list) / traj_list[k] = N by 6 matrix (np.array)
	  trace_time : time series of log file (list)
	  collision : The list of collision log (e.g. [time, collision type, objA objB])
	  collision_pre_list : N by 20 
	  n_obj : The number of objects
	  bool_success : success(True) or failure(false)
	  ID_list : Which ID is in the log file
	'''
	id_grd, s_grd, s_total, id_total, n_total, movable_ID, ID_dict, ID_state_matching = ps.parsing_objects(level_select)		

	with open('{}.log'.format(filename),'rt') as f:
		content = f.readlines()

	print(type(content)) # list
	print(content[0]) #string

	# The number of log lines
	n_log = len(content)

	# Change each string into a list (split it by the whitespace)
	for k in range(n_log):
		content[k] = content[k].split() # content[k] = [5875.626	trace	YT6C	168	-11	0	176	204	0]

	# Split  "trace" case and "collisionStart, collisionEnd"
	trace = []
	collision = []
	collision_pre_list = []
	collision_post_list = []
	# Initialize matrices
	trace_time = []
	collision_start_time = []

	bool_success = False
	for k in range(n_log):
		if content[k][1] == "trace":
			# change string to float for time and states
			for j in [0,3,4,5,6,7,8]:
				content[k][j] = float(content[k][j])
			trace.append(content[k])
		elif content[k][1] == "collisionStart" or content[k][1] == "collisionEnd":		
			# change string to float for time only
			content[k][0] = float(content[k][0])
			collision.append(content[k]) # [time, collision type, objA objB]
		elif content[k][1] == "levelComplete":
			bool_success = True
		else:
			pass


	n_trace = len(trace)
	n_collision = len(collision)


	# Check how many types of objects there are
	n_obj = 1
	count_flag = False
	loop_flag = True
	k = 1
	testID = trace[0][2]
	ID_list = []
	ID_list.append(testID)

	while loop_flag == True:
		temp = trace[k]
		if temp[2] == testID:
			loop_flag = False
		else:
			n_obj = n_obj + 1
			ID_list.append(temp[2])
		k = k+1

	'''
	while loop_flag == True:
		temp = trace[k]
		if count_flag == False:
			# print(temp[1])
			if temp[1] == "trace": # first time to meet 'trace'
				trace_start = k + 1
				count_flag = True
				testID = content[k][2] # allocate testID
				print(testID)			
		else:
			if temp[2] == testID: # check when testID is different from ID		
				loop_flag = False
			else:
				print(temp[2])
				n_obj = n_obj + 1	
		k = k + 1
	'''
	# Create the collison pre_list, collision_post_list
	end_flag = False
	for k in range(n_log):
		if content[k][1] == "collisionStart":
			collision_start_time.append(content[k][0])
			j = k - 1
			# search for 1st correspoding ID's pre/post state	
			if content[k][2] in ID_list:
				# pre
				while content[j][2] != content[k][2]:									
					j = j - 1
				s_pre1 = content[j][3:9] + ID_state_matching['{}'.format(content[k][2])][2:4] #[vx vy w x y theta] + [width height]
				# post
				j = k - 1
				while content[j][2] != content[k][2] and end_flag != True:				
					j = j + 1
					if j >= n_log-1:
						end_flag = True
					elif content[j+1][1] == "levelComplete":
						end_flag = True
				if end_flag == True:
					s_post1 = []
				else:
					s_post1 = content[j][3:9] + ID_state_matching['{}'.format(content[k][2])][2:4] #[vx vy w x y theta] + [width height]
			else:
				# Non-movable object				
				s_pre1 = [0, 0, 0] + ID_state_matching['{}'.format(content[k][2])][0:4]
				s_pre1.insert(5, ID_state_matching['{}'.format(content[k][2])][4])
				#  #[vx vy w] + [x y width height] -> insert
				s_post1 = s_pre1
			# search for 2nd correspoding ID's pre/post state
			j = k - 1
			if content[k][3] in ID_list:
				# pre
				while content[j][2] != content[k][3]:					
					j = j - 1
				s_pre2 = content[j][3:9] + ID_state_matching['{}'.format(content[k][3])][2:4] #[vx vy w x y theta] + [width height]
				# post
				j = k - 1
				while content[j][2] != content[k][3] and end_flag != True:									
					j = j + 1
					if j >= n_log-1:
						end_flag = True
					elif content[j+1][1] == "levelComplete":
						end_flag = True
				if end_flag == True:
					s_post2 = []
				else:
					s_post2 = content[j][3:9] + ID_state_matching['{}'.format(content[k][3])][2:4] #[vx vy w x y theta] + [width height]
			else:
				# Non-movable object				
				s_pre2 = [0, 0, 0] + ID_state_matching['{}'.format(content[k][3])][0:4]
				s_pre2.insert(5, ID_state_matching['{}'.format(content[k][3])][4])
				#  #[vx vy w] + [x y width height] -> insert
				s_post2 = s_pre2
			collision_pre_list.append(s_pre1+s_pre2)
			collision_post_list.append(s_post1+s_post2)
	print(collision_pre_list)

	# make the object's list
	'''
	traj_list[k] = N by 6 matrix (np.array)
	trace_time = N by 1 matrix (list)
	'''

	traj_list = []
	for k in range(n_obj):
		traj_list.append([])

	# sort by ID
	for k in range(0,n_trace,n_obj):
		for i in range(n_obj):
			temp = trace[k+i]
			if temp[1] == "trace":
				traj_list[i].append(temp[3:9])	
		trace_time.append(float(trace[k][0])) # save time as a float (unit : ms)
	# Change the list into the np array
	for k in range(n_obj):
		traj_list[k] = np.array(traj_list[k])

	return 	traj_list, trace_time, collision, n_obj, bool_success, ID_list, collision_pre_list, collision_post_list


if __name__ == "__main__":
	traj_list, trace_time, collision, n_obj, bool_success, ID_list, collision_pre_list, collision_post_list = logging_trajectory("bb", 2)		
	print("ID list is {}".format(ID_list))	
	print("The number of time log is {}".format(len(trace_time)))
	print("The collision list has an element like {}".format(collision[4]))
	print(type(collision[4][0]))
	print("The trace time is {}".format(trace_time[1]))
	print("The traj_list[0] has an element like {}".format(traj_list[0][2]))
	print("collision_pre_list is {}".format(collision_pre_list))
	print(traj_list[0].shape)

