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
      movable_ID : list of movable ID (list) (e.g. ['AO6G', 'C5NY'])
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
            if a[0][0:9] == "metalrect":
                ID_dict['{}'.format(a[6])] = 'metalrectangle'# Insert this into ID_dict
            elif a[0][0:9] == "metalcirc":
                ID_dict['{}'.format(a[6])] = 'metalcircle'
            else:
                ID_dict['{}'.format(a[6])] = 'metalrtriangle'
        elif a[0][0:4] == "wood" or a[0][0:4] =="cata": # catapult has 125x25 size, so we can distinguish
            if a[0][0:7] == "woodrtr":
                n_woodtri += 1
                s_woodtri.append(a[1:6])
                id_woodtri.append(a[6])
                ID_dict['{}'.format(a[6])] = 'woodtriangle'
            elif a[0][0:7] == 'catapul':
                n_wood = n_wood + 1
                s_wood.append(a[1:6])
                id_wood.append(a[6])
                ID_dict['{}'.format(a[6])] = 'catapult'
            else:
                n_wood = n_wood + 1
                s_wood.append(a[1:6])
                id_wood.append(a[6])
                ID_dict['{}'.format(a[6])] = 'woodrectangle'
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
      state_input : which state input you wanna enter ([[id1, x1, y1, theta1],[id2, x2, y2, theta2],...]) list
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
        for obj_input in state_input:
            # 2)-2. Manually input on the script
            input_object["objectSetup"].append({ "unique": "{0}".format(obj_input[0]), "x": int(obj_input[1]), "y": int(obj_input[2]), "r": int(obj_input[3])})
            # input_object["objectSetup"].append({ "unique": "AO6G", "x": 210, "y": 50, "r": 0 })

    # Update the automate.json file with what we wanna enter as an initial configuration
    with open('{}/automate.json'.format(abspath), 'w', encoding = 'utf-8-sig') as automate_file:
        json.dump(input_object, automate_file, indent = 2)
    with open('{}/automate.json'.format(abspath), 'rt', encoding = 'utf-8-sig') as automate_file:
        input_object = json.load(automate_file) 
    obj_init = input_object["objectSetup"][0]
    print("The changed obj_init is {}".format(obj_init))

    # Exectue finally
    #subprocess.run("./bubble-ball")
    #subprocess.run(["ls", "-l"])
    a = subprocess.Popen("./bubble-ball", cwd = abspath)
    time.sleep(10.5)
    a.terminate()
    #gg = input("Check for python works after the bubble-ball execution")



'''
-----------------------------------------------------------------------------------
3) Import json files and parse all objects
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
                    The order of index follows the order of ID_list
      trace_time : time series of log file (list)
      collision : The list of collision log (e.g. [time, collision type, objA objB])
      collision_list :
      n_obj : The number of objects
      bool_success : success(True) or failure(false)
      ID_list : Which ID is in the log file
      collision_pre_list : states before collision  (obj1 state + obj2 state) -> 16 elements
                        [vx vy w x y theta width height]+[vx vy w x y theta width height]
      collision_post_list :  states after collision  (obj1 state + obj2 state)
                        [vx vy w x y theta width height]+[vx vy w x y theta width height]
    '''
    id_grd, s_grd, s_total, id_total, n_total, movable_ID, ID_dict, ID_state_matching = parsing_objects(level_select)        

    with open('{0}/{1}.log'.format(abspath, filename),'rt') as f:
        content = f.readlines()

    print(type(content)) # list
    print(content[0]) #string

    # The number of log lines
    n_log = len(content)

    # Change each string into a list (split it by the whitespace)
    for k in range(n_log):
        content[k] = content[k].split() # content[k] = [5875.626    trace   YT6C    168 -11 0   176 204 0]

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
        if content[k][1] == "collisionStart": #content[k] = [time, collision type, objA objB]
            collision_start_time.append(content[k][0])
            j = k - 1
            # search for 1st correspoding ID's pre/post state   
            if content[k][2] in ID_list:
                while content[j][1] != "trace" and content[j][1] != "levelStart":
                    j -= 1                
                if content[j][1] == "levelStart":
                    state_init = ID_state_matching['{}'.format(content[k][2])]
                    s_pre1 = [0,0,0,state_init[0],state_init[1],state_init[4]] + state_init[2:4] #[vx vy w x y theta] + [width height]    
                else:
                    # pre
                    while content[j][2] != content[k][2]:                                   
                        j = j - 1
                    s_pre1 = content[j][3:9] + ID_state_matching['{}'.format(content[k][2])][2:4] #[vx vy w x y theta] + [width height]
                # post
                j = k - 1 
                while content[j][1] != "trace" and content[j][1] != "levelComplete":
                    j += 1
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
                # Ground objects                
                s_pre1 = [0, 0, 0] + ID_state_matching['{}'.format(content[k][2])][0:4]
                s_pre1.insert(5, ID_state_matching['{}'.format(content[k][2])][4])
                #  [vx vy w] + [x y width height] -> insert the angle later due to the order
                # [vx vy w x y theta width height]
                s_post1 = s_pre1
            # search for 2nd correspoding ID's pre/post state
            j = k - 1
            if content[k][3] in ID_list:
                while content[j][1] != "trace" and content[j][1] != "levelStart":
                    j -= 1                
                if content[j][1] == "levelStart":
                    state_init = ID_state_matching['{}'.format(content[k][2])]
                    s_pre2 = [0,0,0,state_init[0],state_init[1],state_init[4]] + state_init[2:4] #[vx vy w x y theta] + [width height]    
                else:
                    # pre
                    while content[j][2] != content[k][3]:                   
                        j = j - 1
                    s_pre2 = content[j][3:9] + ID_state_matching['{}'.format(content[k][3])][2:4] #[vx vy w x y theta] + [width height]
                # post
                j = k - 1
                while content[j][1] != "trace" and content[j][1] != "levelComplete":
                    j += 1
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

    # make the object's list
    '''
    traj_list[k] = N by 6 matrix (np.array) [vx, vy, w, x, y, rot]
    trace_time = N by 1 matrix (list)
    '''

    traj_list = []
    for k in range(n_obj):
        traj_list.append([])

    # sort by ID
    for k in range(0,n_trace-n_obj,n_obj):
        for i in range(n_obj):
            temp = trace[k+i]
            if temp[1] == "trace":
                traj_list[i].append(temp[3:9])  
        trace_time.append(float(trace[k][0])) # save time as a float (unit : ms)
    # Change the list into the np array
    for k in range(n_obj):
        traj_list[k] = np.array(traj_list[k])

    return  traj_list, trace_time, collision, n_obj, bool_success, ID_list, collision_pre_list, collision_post_list
if __name__ == "__main__":
    level_select = 9
    id_grd, s_grd, s_total, id_total, n_total, movable_ID, ID_dict, ID_state_matching = parsing_objects(level_select)
    print(id_total)
    print(ID_state_matching)
    print(ID_dict)
    print(ID_dict['HG81'])
    # print(id_grd)
    # print("s_grd : {}".format(s_grd))
    # run_simulation(level_select, movable_ID, ID_dict)   

    # For testing logging_traj.
    traj_list, trace_time, collision, n_obj, bool_success, ID_list, collision_pre_list, collision_post_list = logging_trajectory("bb", 2)       
    print(len(traj_list[0]))
    a = traj_list[0]
    b = [a[i][3] for i in range(len(traj_list[0]))] 
    print(b)
    #b = [a[3][j] for j in range(len(a))]
    
    print("ID list is {}".format(ID_list))  
    print("The number of time log is {}".format(len(trace_time)))
    print("The collision list has an element like {}".format(collision[4]))
    print(type(collision[4][0]))
    print("The trace time is {}".format(trace_time[1]))
    print("The traj_list[0] has an element like {}".format(traj_list[0][2]))
    print("collision_pre_list is {}".format(collision_pre_list))
    print(trace_time)
    k = 0
    while abs(4360 - trace_time[k]) > 10:
        k = k+1
    print(k)    
    print(traj_list[0].shape)
    