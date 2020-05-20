# Change the log file to the matrix/vector form

'''
From the log file, we have to get t-vector, state-vector(vx,vy,v_ang,x,y,rotation) of each object
Those objects would be only a ball or wood blocks. It's not necessary to get other things' trajectory since they are stationary. 
e.g. traj1 = N by 7 matrix = [time(1:N).T, state with 6 elements(1:N).T] (traj1 is for each ID)

Thus, after this code runs, we can have n_obj matrices like traj_bal, traj_block1, traj_block2, ...
'''
import numpy as np

with open('bb.log','rt') as f:
	content = f.readlines()

print(type(content))
print(content[0])
print(type(content[0]))
print(len(content))

n_log = len(content)

# Change each string into a list (split it by the whitespace)
for k in range(n_log):
	content[k] = content[k].split()

# Leave only "trace" case
trace = []
for k in range(n_log):
	if content[k][1] == "trace":
		trace.append(content[k])
	else:
		pass

n_trace = len(trace)


# Initialize matrices
time = []
s1 = []

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



# make the object's list
traj_list = []
for k in range(n_obj):
	traj_list.append([])

# sort by ID
for k in range(0,n_trace,n_obj):
	for i in range(n_obj):
		print(k)
		temp = trace[k+i]
		if temp[1] == "trace":
			traj_list[i].append(temp[3:9])	
	time.append(trace[k][0])

print("The number of trace log is {}".format(n_trace))
print("ID list is {}".format(ID_list))	
print("The number of time log is {}".format(len(time)))

# Change the list into the np array
for k in range(n_obj):
	traj_list[k] = np.array(traj_list[k])

print(type(traj_list[2]))
print(traj_list[0])
print(traj_list[0].shape)
			



'''
Output list we can use : time(list), traj_list[k](np array)
'''