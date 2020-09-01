'''
    @ Author : Hotae Lee
    @ Date : 08/12/2020
    @ Function : Find a local region, which is parameterized and able to be adjusted easily 
    @ Parameters : 
    @ Variables:  
    @ Retruns :  
    @ Description : 
    @ TODO : 
'''
import numpy as np
import numdifftools as nd
from matplotlib import pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as trans
from scipy.optimize import minimize, Bounds, LinearConstraint, NonlinearConstraint
import random
from math import cos, sin, pi, sqrt, atan
import os, sys
if __name__ == "__main__":
    from planning_algo.utils import RotatePts, LineInequality, CheckInsideInequality, CheckIntersectInequality, LineFromState, GetDistancePt2Block, GetDistanceBlock2Block, GetFootPerpendicular
    from parsing_movableobjects_levels import parsing_objects, run_simulation, logging_trajectory
    from physics.obj_class import Ball, metalBlock, woodBlock, powerUps
    from physics.simple_models import Ball2LineValue, Ball2CircleValue, BallinAirValue, BallinEnvValue
    from physics.common.const import g, dt
else:
    from . planning_algo.utils import RotatePts, LineInequality, CheckInsideInequality, CheckIntersectInequality, LineFromState, GetDistancePt2Block, GetDistanceBlock2Block, GetFootPerpendicular
    from . parsing_movableobjects_levels import parsing_objects, run_simulation, logging_trajectory
    from . physics.obj_class import Ball, metalBlock, woodBlock, powerUps
    from . physics.simple_models import Ball2LineValue, Ball2CircleValue, BallinAirValue, BallinEnvValue
    from . physics.common.const import g, dt


def SelectLocalRegion(guide_path, x_predicted_traj, y_predicted_traj, s_grd_list, error_threshold = 250):
    # predicted traj : xtraj = [1,2,...], ytraj = [2,3,...]
    # guide_path(shortest_path): [[1,2][2,3]...]
    num_pred_traj = len(x_predicted_traj)
    num_guide_path = len(guide_path)
    dist_list_pred = []
    dist_list = []
    error_list = []
    sum_dist_pred = 0
    sum_dist = 0   
    x_predicted_traj_new = []
    y_predicted_traj_new = []
    # save the intervals of pred_traj and guide_path
    for i in range(num_pred_traj-1):
        xdist_pred = x_predicted_traj[i]-x_predicted_traj[i+1]
        ydist_pred = y_predicted_traj[i]-y_predicted_traj[i+1]
        dist_pred = sqrt(xdist_pred**2+ydist_pred**2)
        sum_dist_pred += dist_pred
        dist_list_pred.append(sum_dist_pred)
    for i in range(num_guide_path-1):
        xdist = guide_path[i][0]-guide_path[i+1][0]
        ydist = guide_path[i][1]-guide_path[i+1][1]
        dist = sqrt(xdist**2+ydist**2)
        sum_dist += dist
        dist_list.append(sum_dist)      

    # pick the pts of predicted traj to compute the tracking error
    # The entire predicted traj. is split into several parts which are proportional to interval of Tg
    for i in range(num_guide_path-1):
        l = dist_list[i]/sum_dist*sum_dist_pred
        id_pick = 1
        while dist_list_pred[id_pick-1]<= l and dist_list_pred[id_pick] > l:
            id_pick += 1
        #id_pick = int(num_pred_traj*dist_list[i]/sum_dist)-1 # consider only the number of elements, not intervals
        x_predicted_traj_new.append(x_predicted_traj[id_pick])
        y_predicted_traj_new.append(y_predicted_traj[id_pick])
        error_list.append(sqrt((guide_path[i][0]-x_predicted_traj_new[i])**2+(guide_path[i][1]-y_predicted_traj_new[i])**2))
    # new index for new list (the number is same with the guide path)
    idx_local_start = 0
    idx_local_end = 0
    print("error list {}".format(error_list))
    print("guide {}".format(guide_path))
    print(x_predicted_traj_new)
    while error_list[idx_local_start] < error_threshold/4:
        idx_local_start += 1
    while error_list[idx_local_end] < error_threshold and idx_local_end < len(error_list)-1:
        idx_local_end += 1
    # Decide the local region
    # id_pick : predicted traj's idx (outside the function, we need to choose s_ball_init)
    #idx_pick_start = int(num_pred_traj*dist_list[idx_local_start]/sum_dist)
    #idx_pick_end = int(num_pred_traj*dist_list[idx_local_end]/sum_dist)
    idx_pick_start = y_predicted_traj.index(y_predicted_traj_new[idx_local_start]) # if it is repeated, return the first value
    idx_pick_end = y_predicted_traj.index(y_predicted_traj_new[idx_local_end])

    guide_path_local = guide_path[idx_local_start:idx_local_end+1] 
    # direction of the end element of local guide path, angle of atan(y/x)
    if guide_path[idx_local_end+1][0]-guide_path[idx_local_end][0] == 0:
        direction_end = pi/2*np.sign(guide_path[idx_local_end+1][1]-guide_path[idx_local_end][1])
    else:
        direction_end = atan((guide_path[idx_local_end+1][1]-guide_path[idx_local_end][1])/(guide_path[idx_local_end+1][0]-guide_path[idx_local_end][0]))
    # It can be changed to the total prediction traj
    x_pred_max = max(x_predicted_traj_new)
    y_pred_max = max(y_predicted_traj_new)
    x_guide_max = guide_path_local[idx_local_end-idx_local_start][0]
    y_guide_max = guide_path_local[idx_local_end-idx_local_start][1]
    x_local_max = x_guide_max
    y_local_max = max(y_pred_max, y_guide_max)
    #x_local = max(x_pred_max,x_guide_max)
    x_local_min = min(guide_path_local[0][0],x_predicted_traj[idx_pick_start])
    y_local_min = min(guide_path_local[0][1],y_predicted_traj[idx_pick_start])
    # Adjust the local region's y value
    s_grd_local = SortoutEnv(x_local_max, y_local_max, x_local_min, y_local_min, s_grd_list) 
    while not s_grd_local or (CheckCondition(s_grd_local, y_pred_max, y_guide_max) and y_local_max <= 500):
        y_local_max += 100
        s_grd_local = SortoutEnv(x_local_max, y_local_max, x_local_min, y_local_min, s_grd_list)

    return guide_path_local, direction_end, x_local_max, y_local_max, idx_pick_start, idx_pick_end

def CheckCondition(s_grd_list, y_pred_max, y_guide_max):
    print(s_grd_list)
    print(max(y_pred_max, y_guide_max))
    bool_condition = True
    for i in range(len(s_grd_list)):
        if s_grd_list[i][1] > max(y_pred_max, y_guide_max) + 50:
            bool_condition = False
    return bool_condition

def SortoutEnv(x_local_max, y_local_max, x_local_min, y_local_min, s_grd_list):
    # ground type is only rectangle, so we don't need ID or type here
    num_grd = len(s_grd_list)
    env_local = []    
    for i in range(num_grd):
        pts = RotatePts(s_grd_list[i], "ground") # we can add groundtriangle
        for pt in pts:
            if pt[0] <= x_local_max and pt[0] >= x_local_min and pt[1]<=y_local_max and pt[1] >=y_local_min:
                env_local.append(s_grd_list[i])
                break
    return env_local

def PickMainBlock(movable_ID, ID_dict, guide_path_local, env_local):
    # for now they just randomly pick one block as a main block
    # We can update this with physics or classification network
    num_movable = len(movable_ID)
    random.shuffle(movable_ID)
    order2pick = movable_ID
    return order2pick

def fobj(x,*y):
    # x is the optimization variable

    # Common things irrelevant to the type of block
    # y[0] = ball.x, y[1] = ball.y, y[2] = ball.vx, y[3] = ball.vy, y[4] = ball.r
    # y[5] = guide_path_local[end][0] : x of guide path, y[6] : y of guide path
    # y[7] : direction of v, vx/vy of guide path
    # y[8] : block type, y[9] : block state [x,y,w,h,rot]

    # tracking error (ref = [x,y,dir])
    ref = [y[5],y[6],y[7]]  

    # All rollouts consist of "gravity or environment -> input -> gravity or environment"
    if y[8] in ("metalrectangle" , "woodrectangle"):
        # 1) retangle
        # x[0] = l, x[1] = theta, x[2] = vx, x[3] = vy, x[4] = w
        #  l should be lower than y[9][3] = w 
        state_ball, _ = Ball2LineValue(y[0],y[1],y[2],y[3],y[4],x[0],x[1],x[2],x[3],x[4])
    elif y[8] == "woodcircle" or y[8] == "metalcircle":
        # 2) circle
        # x[0] = x, x[1] = y, x[2] = vx, x[3] = vy, x[4] = w
        # r is given as y[9][2]/2
        state_ball_collision = Ball2CircleValue(y[0],y[1],y[2],y[3],y[4],y[9][2]/2,x[0],x[1],x[2],x[3],x[4])
        x_distance = abs(ref[0]-y[0])
        y_distance = abs(ref[1]-y[1])
        state_ball = BallinAirValue(state_ball_collision[0],state_ball_collision[1],state_ball_collision[2],state_ball_collision[3],x_distance,y_distance)    
    if abs(state_ball[2]) <0.01: # if ball.vx is too low
        direction = pi/2
    else:
        direction = atan(state_ball[3]/state_ball[2])    
    # error tracking cost
    error = (state_ball[0]-ref[0])**2+(state_ball[1]-ref[1])**2+(direction-ref[2])**2    

    return error

def fobj_constr(x,*y):
    # x is the optimization variable

    # Common things irrelevant to the type of block
    # y[0] = ball.x, y[1] = ball.y, y[2] = ball.vx, y[3] = ball.vy, y[4] = ball.r
    # y[5] = guide_path_local[end][0] : x of guide path, y[6] : y of guide path
    # y[7] : direction of v, vx/vy of guide path
    # y[8] : block type, y[9] : block state [x,y,w,h,rot]
    # y[10] : env_type_pre, y[11] : env_state_pre, y[12] : env_type_post, y[13] : env_state_post
    # y[14] : local_region [x_local_max, y_local_max, xregion, yregion]

    # tracking error (ref = [x,y,dir])
    ref = [y[5],y[6],y[7]]  

    # All rollouts consist of "gravity or environment -> input -> gravity or environment"
    # For f1 (first region before control input)    
    if y[10] == "ground":
        state_ball, _ = Ball2LineValue(y[0],y[1],y[2],y[3],y[4], x[5]/cos(y[11][4]/180*pi), y[11][4], 0, 0, 0)
    else: #elif y[10] == "air":
        if abs(y[2]) < 0.5:
            state_ball = BallinAirValue(y[0],y[1],y[2],y[3],0,x[5])
        else:
            state_ball = BallinAirValue(y[0],y[1],y[2],y[3],x[5])
    xpre = x[5]
    # For fu (control region)
    if y[8] in ("metalrectangle" , "woodrectangle"):
        # 1) retangle
        # x[0] = l, x[1] = theta, x[2] = vx, x[3] = vy, x[4] = w, x[5] = lx (if lx=0, ly)
        #  l should be lower than y[9][3] = w 
        state_ball, _ = Ball2LineValue(state_ball[0],state_ball[1],state_ball[2],state_ball[3],y[4],x[0],x[1],x[2],x[3],x[4])
        xu = x[0]*cos(x[1]/180*pi)
    elif y[8] == "woodcircle" or y[8] == "metalcircle":
        # 2) circle
        # x[0] = x, x[1] = y, x[2] = vx, x[3] = vy, x[4] = w
        # r is given as y[9][2]/2
        state_ball_collision = Ball2CircleValue(state_ball[0],state_ball[1],state_ball[2],state_ball[3],y[4],y[9][2]/2,x[0],x[1],x[2],x[3],x[4])
        x_distance = abs(ref[0]-y[0])
        y_distance = abs(ref[1]-y[1])
        state_ball = BallinAirValue(state_ball_collision[0],state_ball_collision[1],state_ball_collision[2],state_ball_collision[3],x_distance,y_distance)    
        xu = y[9][2]
    # For f2 (last region after control input)
    xpost = y[14][0] - xu - xpre  
    # if y[12] == "ground":
    #     state_ball, _ = Ball2LineValue(state_ball[0],state_ball[1],state_ball[2],state_ball[3], y[4], xpost/cos(y[13][4]/180*pi), y[13][4], 0, 0, 0)
    # else: #elif y[12] == "air": we need to decide more expressive functions based on observations 
    #     state_ball= BallinAirValue(state_ball[0],state_ball[1],state_ball[2],state_ball[3],xpost)

    if abs(state_ball[2]) <0.01: # if ball.vx is too low
        direction = pi/2
    else:
        direction = atan(state_ball[3]/state_ball[2])    
    # error tracking cost
    error = (state_ball[0]-ref[0])**2+(state_ball[1]-ref[1])**2+(direction-ref[2])**2    

    return error

def fmodel(x,*y):
    # x is the optimization variable

    # Common things irrelevant to the type of block
    # y[0] = ball.x, y[1] = ball.y, y[2] = ball.vx, y[3] = ball.vy, y[4] = ball.r
    # y[5] = guide_path_local[end][0] : x of guide path, y[6] : y of guide path
    # y[7] : direction of v, vx/vy of guide path
    # y[8] : block type, y[9] : block state [x,y,w,h,rot]

    # tracking error (ref = [x,y,dir])
    ref = [y[5],y[6],y[7]]  

    # All rollouts consist of "gravity or environment -> input -> gravity or environment"
    if y[8] in ("metalrectangle" , "woodrectangle"):
        # 1) retangle
        # x[0] = l, x[1] = theta, x[2] = vx, x[3] = vy, x[4] = w
        #  l should be lower than y[9][3] = w 
        state_ball, _ = Ball2LineValue(y[0],y[1],y[2],y[3],y[4],x[0],x[1],x[2],x[3],x[4])
    elif y[8] == "woodcircle" or y[8] == "metalcircle":
        # 2) circle
        # x[0] = x, x[1] = y, x[2] = vx, x[3] = vy, x[4] = w
        # r is given as y[9][2]/2
        state_ball_collision = Ball2CircleValue(y[0],y[1],y[2],y[3],y[4],y[9][2]/2,x[0],x[1],x[2],x[3],x[4])
        x_distance = abs(ref[0]-y[0])
        y_distance = abs(ref[1]-y[1])
        state_ball = BallinAirValue(state_ball_collision[0],state_ball_collision[1],state_ball_collision[2],state_ball_collision[3],x_distance,y_distance)    
    if abs(state_ball[2]) <0.01: # if ball.vx is too low
        direction = pi/2
    else:
        direction = atan(state_ball[3]/state_ball[2])    
    result = np.array([state_ball[0],state_ball[1],direction])
    return result

def ConvertUopt2Ureal(u_input, block_type, w_block, h_block, s_ball_ini):    
    xball = s_ball_ini[0]
    yball = s_ball_ini[1]
    vxball = s_ball_ini[2]
    vyball = s_ball_ini[3]
    rball = s_ball_ini[4]
    if block_type in ("metalrectangle" , "woodrectangle"):
        # u_input = [ l, theta, vx, vy, w] + lx
        l = u_input[0]
        width = w_block
        height = h_block
        theta = u_input[1]/180*pi
        u_actual = [xball+rball-rball*sin(theta)+l*cos(theta)-width/2-width/2*cos(theta)-height/2*sin(theta), yball+rball+rball*cos(theta)+l*sin(theta)-height/2-width/2*sin(theta)+height/2*cos(theta), u_input[1]]
        vel_desired = [u_input[2],u_input[3],u_input[4]]
    elif block_type == "woodcircle" or block_type ==  "metalcircle":
        # u_input = [x, y, vx, vy, w]
        u_actual = [u_input[0],u_input[1],0]
        vel_desired = [u_input[2],u_input[3],u_input[4]]
    elif block_type in ("metalrtriangle", "woodrtriangle"):
        # u_input = [ l, theta, vx, vy, w]   
        l = u_input[0]     
        theta = (u_input[1]+45)/180*pi
        u_actual = [xball-(w_block*sqrt(2)-l)*cos(theta), yball+ rball*2, theta]
    return u_actual, vel_desired

def FindOptimalInput(guide_path_local, direction_end, block_type, block_state, s_ball_ini):
    # the size of the local region and the environment in the local region can be input
    # But, we want to minimize the intervention of environment.
    # Input : guide_path_local, block_type, block_state[x,y,w,h,rot], s_ball_ini (x,y,vx,vy,r)
    # Output : optimal inputs to minize the tracking error based on the simple model
    xball = s_ball_ini[0]
    yball = s_ball_ini[1]
    vxball = s_ball_ini[2]
    vyball = s_ball_ini[3]
    rball = s_ball_ini[4]
    # ref : [x,y,dir]
    ref = (guide_path_local[-1][0],guide_path_local[-1][1], direction_end)
    # Given parameters
    # y = (ball(5),ref(3),block type, block state)
    y = (s_ball_ini[0],s_ball_ini[1],s_ball_ini[2],s_ball_ini[3],s_ball_ini[4], ref[0],ref[1],ref[2], block_type, block_state)        
    if block_type == "metalrectangle" or block_type == "woodrectangle":
        # x[0] = l, x[1] = theta, x[2] = vx, x[3] = vy, x[4] = w
        # block_state = [x,y,w,h,rot]        
        x0 = [block_state[2],2,0,0,0]        
        # bound : 0 < l < width, -90<theta<90
        search_bound = Bounds([0,-90,-np.inf,-np.inf,-np.inf],[block_state[2],90,np.inf,np.inf,np.inf])
        # nonlinear constr : l and theta have same sign
        f_nonlin = lambda x:x[0]*x[1]
        nonlin_constr = NonlinearConstraint(f_nonlin,0,np.inf)
        # solve optimization
        res = minimize(fobj ,x0, args = y, method ='trust-constr' , bounds = search_bound, constraints = nonlin_constr)
        print(res)                
        u_input = res.x
    elif block_type == "woodcircle" or block_type == "metalcircle":
        # x[0] = x, x[1] = y, x[2] = vx, x[3] = vy, x[4] = w
        x0 = [xball+block_state[2]/2,yball+block_state[2]/2,0,0,0]
        # bound : we need a condition to collide and x,y should be inside the local region
        search_bound = Bounds([0,0,-np.inf,-np.inf,-np.inf],[ref[0],ref[1],np.inf,np.inf,np.inf])
        # nonlinear constr : nothing now
        #f_nonlin = lambda x:x[0]*x[1]
        #nonlin_constr = NonlinearConstraint(f_nonlin,0,np.inf)
        # solve optimization
        res = minimize(fobj ,x0, args = y, bounds = search_bound)
        print(res)
        u_input = res.x
    return u_input

def FindOptimalInputConstrained(guide_path_local, direction_end, block_type, block_state, s_ball_ini, env_type_pre, env_state_pre, env_type_post, env_state_post, env_local, local_region):
    # the size of the local region and the environment in the local region can be input
    # But, we want to minimize the intervention of environment.
    # Input : guide_path_local, block_type, block_state[x,y,w,h,rot], s_ball_ini (x,y,vx,vy,r), 
    #         evn_type ("ground" or "air")
    #         env_local : [[x,y,w,h,rot],[x,y,w,h,rot],..], local_region : [x_local_max, y_local_max, xregion, yregion]
    # Output : optimal inputs to minize the tracking error based on the simple model    
    xball = s_ball_ini[0]
    yball = s_ball_ini[1]
    vxball = s_ball_ini[2]
    vyball = s_ball_ini[3]
    rball = s_ball_ini[4]
    # ref : [x,y,dir]
    ref = (guide_path_local[-1][0],guide_path_local[-1][1], direction_end)
    constr_list = []
    # Given parameters
    # y = (ball(5),ref(3),block type, block state, env_type_pre, env_state_pre, env_type_post, env_state_post, localregion(4))
    y = (s_ball_ini[0],s_ball_ini[1],s_ball_ini[2],s_ball_ini[3],s_ball_ini[4], ref[0],ref[1],ref[2], block_type, block_state, env_type_pre, env_state_pre, env_type_post, env_state_post, local_region)        
    if block_type == "metalrectangle" or block_type == "woodrectangle":
        # x[0] = l, x[1] = theta, x[2] = vx, x[3] = vy, x[4] = w / x[5] = lx for s_mid = f1(s_in, lx) (lx = 0, ly)
        def pts_block(x):       
            if env_type_pre == "ground":
                state_ball, _ = Ball2LineValue(y[0],y[1],y[2],y[3],y[4], x[5]/cos(y[11][4]/180*pi), y[11][4], 0, 0, 0)
            else: #elif y[10] == "air":
                if abs(y[2]) < 0.5:
                    state_ball = BallinAirValue(y[0],y[1],y[2],y[3],0,x[5])
                else:
                    state_ball = BallinAirValue(y[0],y[1],y[2],y[3],x[5])     
            l = x[0]
            theta = x[1]/180*pi
            width = block_state[2]
            height = block_state[3]
            u_actual = [state_ball[0]+rball-rball*sin(theta)+l*cos(theta)-width/2-width/2*cos(theta)-height/2*sin(theta), state_ball[1]+rball+rball*cos(theta)+l*sin(theta)-height/2-width/2*sin(theta)+height/2*cos(theta), x[1]]
            pts_block = RotatePts([u_actual[0],u_actual[1],width, height, u_actual[2]], block_type)
            return pts_block
        if abs(vxball) < 0.5:
            # x[5] = ly
            x0 = [block_state[2],0,0,0,0, (y_local_max-yregion)/10]     
            search_bound = Bounds([0,-90,-np.inf,-np.inf,-np.inf, 0],[block_state[2],90,np.inf,np.inf,np.inf, y_local_max-yregion])
        else:
            # block_state = [x,y,w,h,rot]        
            x0 = [block_state[2],0,0,0,0, (x_local_max-xregion)/4]        
            # bound : 0 < l < width, -90<theta<90
            search_bound = Bounds([0,-90,-np.inf,-np.inf,-np.inf, 0],[block_state[2],90,np.inf,np.inf,np.inf, x_local_max-xregion])
        # nonlinear constr : l and theta have same sign
        f_nonlin = lambda x:x[0]*x[1]
        nonlin_constr = NonlinearConstraint(f_nonlin,0,np.inf)
        constr_list.append(nonlin_constr)
        # process constraint
        # if env_type_pre == "ground":
        #     state_ball, _ = Ball2LineValue(y[0],y[1],y[2],y[3],y[4], x[5]/cos(y[11][4]/180*pi), y[11][4], 0, 0, 0)
        # else: #elif y[10] == "air":
        #     state_ball, _ = BallinAirValue(y[0],y[1],y[2],y[3],x[5])

        # Interference constraint        
        for env_element in env_local:
            # for radius (size growing downwards)
            # to avoid referring to the address of list directly
            width_element = env_element[2] + 30
            height_element = env_element[3] + 30
            state_element = [env_element[0], env_element[1], width_element, height_element, env_element[4]]
            pts = RotatePts(state_element, "ground")
            # for i in range(4):
            #     f_interference1 = lambda x: CheckInsideInequality(pts, pts_block(x)[i])
            #     f_interference2 = lambda x: CheckInsideInequality(pts_block(x), pts[i])
            #     constr_interference1 = NonlinearConstraint(f_interference1, -np.inf, 0)
            #     constr_interference2 = NonlinearConstraint(f_interference2, -np.inf, 0)
            #     constr_list.append(constr_interference1)
            #     constr_list.append(constr_interference2)
            # for i in range(4):
            #     for j in range(4):
            #         f_interference3 = lambda x: CheckIntersectInequality(pts_block(x)[i], pts_block(x)[i-1], pts[j], pts[j-1])
            #         constr_interference3 = NonlinearConstraint(f_interference3, -np.inf, 0)
            #         constr_list.append(constr_interference3)

        # pts = RotatePts(env_local[2], "ground")
        # f_test = lambda x: LineInequality(pts[1],pts[2],pts_block(x)[0])
        # constr_test = NonlinearConstraint(f_test, -np.inf, 0)
        # constr_list.append(constr_test)
        # f_test = lambda x: [pts_block(x)[0][0]-450, pts_block(x)[1][0]-450, pts_block(x)[2][0]-450]
        # constr_test = NonlinearConstraint(f_test, -np.inf, 0)
        # constr_list.append(constr_test)
        # f_test = lambda x : 2-x[1]
        # constr_test = NonlinearConstraint(f_test, -np.inf, 0)
        # constr_list.append(constr_test)
        # solve optimization
        res = minimize(fobj_constr ,x0, args = y, bounds = search_bound, constraints = constr_list)
        print(res)                
        u_input = res.x
    elif block_type == "woodcircle" or block_type == "metalcircle":
        # x[0] = x, x[1] = y, x[2] = vx, x[3] = vy, x[4] = w
        x0 = [xball+block_state[2]/2,yball+block_state[2]/2,0,0,0]
        # bound : we need a condition to collide and x,y should be inside the local region
        search_bound = Bounds([0,0,-np.inf,-np.inf,-np.inf],[ref[0],ref[1],np.inf,np.inf,np.inf])
        # nonlinear constr : nothing now
        #f_nonlin = lambda x:x[0]*x[1]
        #nonlin_constr = NonlinearConstraint(f_nonlin,0,np.inf)
        # solve optimization
        res = minimize(fobj_constr ,x0, args = y, bounds = search_bound)
        print(res)
        u_input = res.x
    return u_input

def ComputeSupportForces(mainblock_type, mainblock_desired_state, main_traj,  contact_idx):
    # Compute the 2 forces or 1 force with a fixed force, given the desired state of a main block
    # Input : type("rectangle"), desired_state(x,y,w,h,rot,vx,vy,w_rot)
    #         current trajectory : main_traj([[vx,vy,w,x,y,rot],[],...,[]])
    #         contact_point : [x,y], contact_idx : index of traj. when the contact starts
    # Output : two points we will support (p1[x1,y1], p2[x2,y2]) / one point or range corresponding to given contact pt
    # the supporting forces which can compensate the movement
    if mainblock_type == "woodrectangle":
        m = 1.5*mainblock_desired_state[2]*mainblock_desired_state[3]
        I = 1/12*m*(mainblock_desired_state[2]**2+mainblock_desired_state[3]**2)
    elif mainblock_type == "woodcircle":
        m = 1.5*pi*(mainblock_desired_state[3]/2)**2
        I = 1/2*m*(mainblock_desired_state[3]/2)**2
    elif mainblock_type == "woodrtriangle":
        m = 1/2*1.5*mainblock_desired_state[2]*mainblock_desired_state[3]
        I = 1/9*m*mainblock_desired_state[3]**2
    # Compute the current forces under this traj.    
    # If we observed main_traj, we can compute force traj.
    fx_list = []
    fy_list = []
    torque_list = []
    for i in range(len(main_traj)-1):
        fx = m*(main_traj[i+1][0]-main_traj[i][0])/dt         
        fy = m*(main_traj[i+1][1]-main_traj[i][1])/dt
        torque = I*(main_traj[i+1][2]-main_traj[i][2])/dt
        fx_list.append(fx)
        fy_list.append(fy)
        torque_list.append(torque)
    # Actual forces/torques when contact starts
    fx1 = fx_list[contact_idx]
    fy1 = fy_list[contact_idx]
    torque1 = torque_list[contact_idx]
    # Compute the desired force/torque (F = m(vref-0)/dt)
    fx_desired = m*mainblock_desired_state[5]/dt
    fy_desired = m*mainblock_desired_state[6]/dt
    torque_desired = I*mainblock_desired_state[7]/dt
    fx2 = fx_desired - fx1 
    fy2 = fy_desired - fy1 - m*g
    torque2 = torque_desired - torque1
    # roughly compute (xcm+lx, ycm)
    if fx2 == 0 and fy2 == 0:
        p2 = None
    else:
        lx = torque2/sqrt(fx2**2+fy2**2)
        theta = mainblock_desired_state[4]/180*pi
        x2 = mainblock_desired_state[0] + mainblock_desired_state[2]/2 + lx*cos(theta) + mainblock_desired_state[3]/2*sin(theta)
        y2 = mainblock_desired_state[1] + mainblock_desired_state[3]/2 + lx*sin(theta) + mainblock_desired_state[3]/2*cos(theta)
        # we need the function to find the intersection 
        p2 = [x2,y2]
    return p2

def UpdateModelAfterFailure(guide_path_local, direction_end, block_type, block_state, s_ball_ini, u_pre, f_actual):
    # Update the mainblock's state after observations
    # Input : mainblock_type, previous_input, f_actual (x,y,dir) / env, xstart
    # Output : next_input or new model parameter

    # Model error term's update
    model_paramter = 0
    # ref : [x,y,dir]
    ref = np.array((guide_path_local[-1][0],guide_path_local[-1][1], direction_end))
    # y = (ball(5),ref(3),block type, block state)
    y = (s_ball_ini[0],s_ball_ini[1],s_ball_ini[2],s_ball_ini[3],s_ball_ini[4], ref[0],ref[1],ref[2], block_type, block_state)  

    # Approximate Gradient descent 
    alpha = 0.3
    jac = nd.Jacobian(fmodel)
    a = jac(u_pre, *y)
    b = a.T
    unew = u_pre - alpha*(2*np.matmul(b,f_actual)-2*np.matmul(b,ref))    
    return unew

def UpdateCostWeights():
    weights = [0,0,1]
    return weights

class Drawing:
    def __init__(self, xregion, yregion, xsize, ysize, s_grd_list, guide_path, guide_path_local, s_ball_ini):
        self.xregion = xregion
        self.yregion = yregion
        self.xsize = xsize
        self.ysize = ysize
        self.s_grd_list = s_grd_list
        self.guide_path = guide_path
        self.guide_path_local = guide_path_local
        self.s_ball_ini = s_ball_ini
    def drawlocalregion(self):
        # Input : xregion,yregion,xsize,ysize / s_grd_list,  s_ball_ini (x,y,vx,vy,r), guide_path_local
        # guide_path_local (from shortest path) : [[1,2],[2,3],...]
        '''
        ---------------------------------------------------------------------
        Draw the figure (We can make this be a function)
        ---------------------------------------------------------------------
        '''
        # Draw path planning figure
        fig1, self.ax1 = plt.subplots()
        self.ax1.axis('scaled')
        self.ax1.set(xlim = (0-1,500+1), ylim = (500+1,0-1))
        
        # Draw the red local region boundary
        local = patches.Rectangle((self.xregion,self.yregion),self.xsize,self.ysize,edgecolor='r', facecolor="none") 
        self.ax1.add_patch(local)

        # Prepare the PRM figure
        xpath = [self.guide_path[i][0] for i in range(len(self.guide_path))]
        ypath = [self.guide_path[i][1] for i in range(len(self.guide_path))]
        # Prepare the PRM figure
        xpath_local = [self.guide_path_local[i][0] for i in range(len(self.guide_path_local))]
        ypath_local = [self.guide_path_local[i][1] for i in range(len(self.guide_path_local))]
        # Draw the shortest path [[1,2],[2,3],...]
        self.ax1.plot(xpath, ypath, c = 'c', marker = 'o')
        self.ax1.plot(xpath_local,ypath_local, c = 'b', marker = 'o')

        # Draw the grounds and the ball / flag
        for s_grd in self.s_grd_list:
            ts = self.ax1.transData
            tr = trans.Affine2D().rotate_deg_around(s_grd[0]+s_grd[2]/2,s_grd[1]+s_grd[3]/2, s_grd[4])
            t = tr + ts # tr + ts (order is important)
            rect = patches.Rectangle((s_grd[0],s_grd[1]),s_grd[2],s_grd[3], edgecolor='k', facecolor="k", transform = t)
            self.ax1.add_patch(rect)
        ball = plt.Circle((self.s_ball_ini[0]+15,self.s_ball_ini[1]+15),self.s_ball_ini[4], facecolor = 'orange') # +15 means the ball's center
        self.ax1.add_artist(ball)
        self.ax1.plot()
        return self.ax1
    def drawobject(self, obj):
        if obj.__class__.__name__ == "woodBlock":
            # we have to differentiate block_type
            ts = self.ax1.transData
            tr = trans.Affine2D().rotate_deg_around(obj.x+obj.w/2,obj.y+obj.h/2, obj.rot)
            t = tr + ts # tr + ts (order is important)
            rect = patches.Rectangle((obj.x,obj.y),obj.w,obj.h, edgecolor='y', facecolor="y", transform = t)
            self.ax1.add_patch(rect) 
            self.ax1.plot()
        elif obj.__class__.__name__=="metalBlock":
            ts = self.ax1.transData
            tr = trans.Affine2D().rotate_deg_around(obj.x+obj.w/2,obj.y+obj.h/2, obj.rot)
            t = tr + ts # tr + ts (order is important)
            rect = patches.Rectangle((obj.x,obj.y),obj.w,obj.h, edgecolor='gray', facecolor="gray", transform = t)
            self.ax1.add_patch(rect) 
            self.ax1.plot()
        return self.ax1

def ff(x,*y):    
    a = x[0]
    b = x[1]**2+y[0]*x[1]+y[1]
    r = np.array([a,b])
    return r

def MoveBlock2CloestBlock(block_state, block_type, env_local, env_type_list = []):
    # Check the distance between the optimal main block and the ground
    dist_min = 500
    dist_vector_min = [0,0]
    for s_grd_local in env_local:
        dist_vector, dist = GetDistanceBlock2Block(block_state, s_grd_local, block_type, 'ground')
        if dist < dist_min:
            dist_min = dist
            dist_vector_min = dist_vector           
    # Move the block to the point which is closest to the main block
    u_move = [block_state[0]+dist_vector_min[0], block_state[1]+dist_vector_min[1], block_state[4]]
    print("dist min{}".format(dist_min))
    return u_move


if __name__=="__main__":
    # s_ball_ini = [60,80,0,5,15] # s_ball_ini (x,y,vx,vy,r)
    # ref = [100,100,0.3]
    # block_type = "rectangle"
    # block_state = [0,0,50,150,0]
    # model_error = np.array([10,10,0.1])
    # # FindOptimalInput(guide_path_local, direction_end, block_type, block_state, s_ball_ini):
    # u = FindOptimalInput([[60,80],[100,100]], 0.3, "rectangle", [0,0,50,150,0],[60,80,0,5,15])
    # #u = FindOptimalInput([[60,80],[100,100]], 0.3, "circle", [0,0,50,50,0],[60,80,0,5,15])
    # print("u is {}".format(u))
    # y = (s_ball_ini[0],s_ball_ini[1],s_ball_ini[2],s_ball_ini[3],s_ball_ini[4], ref[0],ref[1],ref[2], block_type, block_state)
    # f_actual = fmodel(u,*y)+model_error
    # error = ref - f_actual
    # e = np.matmul(error,error)
    # print("error is {}".format(e))
    # print(f_actual)
    # #a = np.array([1,2])
    # #print(ff(a,*(2,3)))
    # #f = nd.Jacobian(ff)
    # #print(f(a,*(4,3)))
    # unew = UpdateModelAfterFailure([[60,80],[100,100]], 0.3, "rectangle", [0,0,50,150,0],[60,80,0,5,15], u, f_actual)
    # print("new u is {}".format(unew))
    # f_new = fmodel(unew,*y)+model_error
    # print(f_new)
    # unew1 = UpdateModelAfterFailure([[60,80],[100,100]], 0.3, "rectangle", [0,0,50,150,0],[60,80,0,5,15], unew, f_new)
    # print("new u is {}".format(unew1))
    # f_new1 = fmodel(unew1,*y)+model_error
    # print(f_new1)

    # # Draw local region 
    # xregion = 100
    # yregion = 100
    # xsize = 200
    # ysize = 150
    # s_grd_list = [[60,150,80,10,0],[150,170,90,15,10]]
    # guide_path_local = [[100,100],[150,110],[200,150],[300,160]]
    # s_ball_ini = [100,100,10,0,15]

    # # woodblock state
    # w = 100
    # h = 20

    # x = 150
    # y = 130
    # rot = 5
    # vx = 0
    # vy = 0
    # w_rot = 0
    # block_type = "rectangle"

    # block1 = woodBlock(x, y, w, h, rot, vx, vy, w_rot, block_type)
    # display = Drawing(xregion, yregion, xsize, ysize, s_grd_list, guide_path_local, s_ball_ini)
    # display.drawlocalregion()
    # display.drawobject(block1)
    # plt.show()


    # Test for a local region
    level_select = 4
    # 0) Prerequisite : guide path
    #guide_path = [[20,135],[50,155],[110,150],[180,170],[230,185],[280,175],[310,185],[340,195],[400,200]]
    guide_path = [[419, 285], [439, 254], [440, 240], [424, 217], [383, 195], [338, 154], [331, 153], [290, 169], [266, 133], [230, 153], [220, 141], [178, 144], [142, 136], [120, 126], [99, 120], [75, 100]]
    guide_path.reverse()
    #guide_path = [[210, 100], [241, 119], [284, 139], [304, 129], [348, 137], [372, 144], [391, 151], [435, 153], [476, 167], [446, 210], [407, 223], [389, 227], [377, 260], [345, 276], [310, 274], [276, 263], [255, 283], [243, 287], [209, 294], [176, 288], [159, 292], [139, 293], [117, 300], [86, 302], [59, 292], [34, 285]]
    state_input = [] # decided ahead in previous local regions
    #state_input.append(["AO6G", 0,0,0])
    
    # 1) Predict the ball's traj, load a traj. from a log file
    id_grd, s_grd_list, s_total, id_total, n_total, movable_ID, ID_dict, ID_state_matching = parsing_objects(level_select)
    state_input.append(["{}".format(movable_ID[0]), 0,0,0])
    #run_simulation(level_select, movable_ID, ID_dict, state_input)
    traj_list, trace_time, collision, n_obj, bool_success, ID_list, _, _ = logging_trajectory("bb", level_select)       
    id_ball = id_total[0]
    s_ball = s_total[0] # [x,y,w,h,rot], we use only w and h here
    # check the order of the ID in the ID_list
    ball_idx = 0
    while id_ball != ID_list[ball_idx] and ball_idx < len(ID_list):
        ball_idx = ball_idx+1
    ball_traj = traj_list[ball_idx] # [vx,vy,wrot,x,y,theta] 6 by N
    
    # 2) Choose a local region and cut-off the guide path locally
    x_predicted_traj = [ball_traj[j][3] for j in range(len(ball_traj))]
    y_predicted_traj = [ball_traj[j][4] for j in range(len(ball_traj))]
        
    guide_path_local, direction_end, x_local_max, y_local_max, idx_pick_start, idx_pick_end = SelectLocalRegion(guide_path, x_predicted_traj, y_predicted_traj, s_grd_list)
    xregion = min(guide_path_local[0][0],x_predicted_traj[idx_pick_start])
    yregion = min(guide_path_local[0][1],y_predicted_traj[idx_pick_start])
    xsize = x_local_max-xregion
    ysize = y_local_max-yregion
    local_region = [x_local_max, y_local_max, xregion, yregion]
    # s_ball_ini = [x,y,vx,vy,r] : so we have to change the order because log file has vx,vy first
    s_ball_ini = [ball_traj[idx_pick_start][3],ball_traj[idx_pick_start][4],ball_traj[idx_pick_start][0],ball_traj[idx_pick_start][1],15]
    # sort out the grounds' states from the total s_grd_list
    env_local = SortoutEnv(x_local_max, y_local_max, xregion, yregion, s_grd_list)
    # get the types and states of envs
    env_left = [500,0,0,0,0]
    env_right = [0,0,0,0,0]
    for env_element in env_local:
        if env_element[0] < env_left[0]:
            env_left = env_element
    for env_element in env_local:
        if env_element[0] > env_right[0]:
            env_right = env_element
    if s_ball_ini[2] >= 0: # go to the right
        env_type_pre = "air"
        env_state_pre = env_left
        env_type_post = "air"
        env_state_post = env_right
    else:
        env_type_pre = "air"
        env_state_pre = env_right
        env_type_post = "air"
        env_state_post = env_left
    print("envlocal:{}".format(env_local))
    print(env_left, env_right)

    display = Drawing(xregion, yregion, xsize, ysize, s_grd_list, guide_path, guide_path_local, s_ball_ini)
    display.drawlocalregion()
    plt.show(block=False)
    plt.pause(2)    

    # While the local region fails, do a loop
    num_movable = len(movable_ID)
    pick_main_idx = 0
    
    # 3) Pick a main block / we can add some priorities    
    input_id = movable_ID[pick_main_idx]
    block_type = ID_dict[input_id] # 'metalrectangle'
    block_state = ID_state_matching[input_id] # [x,y,w,h,rot]
    print("guide_path_local {}".format(guide_path_local))

    # 4) Solve the optimization problem to obtain the required state of the main block
    u_optimal = FindOptimalInput(guide_path_local, direction_end, block_type, block_state, s_ball_ini)
    #u_optimal = FindOptimalInputConstrained(guide_path_local, direction_end, block_type, block_state, s_ball_ini, env_type_pre, env_state_pre, env_type_post, env_state_post, env_local, local_region)
    w_block = block_state[2]
    h_block = block_state[3]
    # after f1, get the initial state just before entering fu
    # We can use the actual data and get the y value at x(u_optimal[5])
    # if env_type_pre == "ground":
    #     state_ball, _ = Ball2LineValue(s_ball_ini[0],s_ball_ini[1],s_ball_ini[2],s_ball_ini[3],s_ball_ini[4], u_optimal[5]/cos(env_state_pre[4]/180*pi), env_state_pre[4], 0, 0, 0)
    # else: #elif y[10] == "air":
    #     if abs(s_ball_ini[2]) < 0.5:
    #         state_ball = BallinAirValue(s_ball_ini[0],s_ball_ini[1],s_ball_ini[2],s_ball_ini[3],0,u_optimal[5])
    #     else:
    #         state_ball = BallinAirValue(s_ball_ini[0],s_ball_ini[1],s_ball_ini[2],s_ball_ini[3],u_optimal[5])
    # s_ball_mid = [state_ball[0], state_ball[1], state_ball[2], state_ball[3], s_ball_ini[4]]
    # u_actual, vel_desired = ConvertUopt2Ureal(u_optimal, block_type, w_block, h_block, s_ball_mid)
    u_actual, vel_desired = ConvertUopt2Ureal(u_optimal, block_type, w_block, h_block, s_ball_ini)
    # print("sball ini, sball mid {0}, {1}".format(s_ball_ini, s_ball_mid))
    block_state = [u_actual[0], u_actual[1], w_block, h_block, u_actual[2]]
    # Move the block to the point which is closest to the main block
    u_move = MoveBlock2CloestBlock(block_state, block_type, env_local, env_type_list = [])
    print("u_actual, u_move")
    print(u_actual, u_move)
    block_state_move = [u_move[0], u_move[1], w_block, h_block, u_move[2]]
    # Decide the region below the main block (roughly we can use [x,y,x+w,ymax])
    pts_move = RotatePts(block_state_move, block_type)
    x_move_list = [pts_move[i][0] for i in range(len(pts_move))]
    y_move_list = [pts_move[i][1] for i in range(len(pts_move))]
    x_move_min = min(x_move_list)
    y_move_min = min(y_move_list)
    x_move_max = max(x_move_list)
    y_move_max = 500

    # Check which grounds exist below the mainblock now among local_env
    ground_below = SortoutEnv(x_move_max, y_move_max, x_move_min, y_move_min, env_local)
    # Check the other point's shortest distance upto the ground
    if u_move[0] >= u_actual[0]:
        # It means the block move to the right. we will check left point
        pt_left = pts_move[3]
        dist_min = 500
        pt_min = []
        for grd in ground_below:
            dist_below, pt_below = GetDistancePt2Block(pt_left, grd, 'ground')
            if dist_below < dist_min:
                dist_min = dist_below
                pt_min = pt_below
                grd_choice = grd
    else:
        pt_right = pts_move[0]
        dist_min = 500
        for grd in ground_below:
            dist_below, pt_below = GetDistancePt2Block(pt_right, grd, 'ground')
            if dist_below < dist_min:
                dist_min = dist_below
                pt_min = pt_below
                grd_choice = grd
    # Compare the distance with some sizes of block
    idx_order_from_closest_to_farthest = []
    gap_list = []
    if dist_min < 50 or not pt_min:
        pass
    else:
        for pick_support_idx in range(num_movable):
            if pick_support_idx == pick_main_idx:
                pass
            else:
                # Pick supporting blocks / we can add some priorities    
                support_id = movable_ID[pick_support_idx]
                support_block_type = ID_dict[support_id] # 'metalrectangle'
                support_block_state = ID_state_matching[support_id] # [x,y,w,h,rot]
                w_temp = support_block_state[2]
                h_temp = support_block_state[3]
                gap_temp = min(abs(dist_min-w_temp), abs(dist_min-h_temp))
                if not gap_list:
                    gap_list.append(gap_temp)
                    idx_order_from_closest_to_farthest.append(pick_support_idx)
                else:
                    for j in range(len(gap_list)):
                        if gap_list[j] >= gap_temp:
                            gap_list.insert(j, gap_temp)
                            idx_order_from_closest_to_farthest.insert(j, pick_support_idx)
    print(idx_order_from_closest_to_farthest)

    # desired_state(x,y,w,h,rot,vx,vy,w_rot)
    mainblock_desired_state = [u_move[0],u_move[1],w_block,h_block,u_move[2],vel_desired[0],vel_desired[1],vel_desired[2]]
    if not idx_order_from_closest_to_farthest:
        support_id = None
        support_type = None
    else:
        # rough guess of configuration of supporting blocks
        support_idx = idx_order_from_closest_to_farthest[0]
        support_id = movable_ID[support_idx]
        support_type = ID_dict[support_id] # 'metalrectangle'
        support_state = ID_state_matching[support_id] # [x,y,w,h,rot]
        w_support = support_state[2]
        h_support = support_state[3]
        if u_move[0] >= u_actual[0]: # move right, left point is left
            x_support = pt_min[0]
            y_support = pt_min[1]-h_support
        else:
            x_support = pt_min[0]-w_support/2
            y_support = pt_min[1]-h_support
        theta_support = grd_choice[4] # same with the angle of underlying ground

    # 5) Simulate the main block and observe the traj. with roughly estimated supporting blocks
    # state_input = ([[id1, x1, y1, theta1],[id2, x2, y2, theta2],...]) list
    # previous input is removed but, we have to leave the previous local region's input
    del state_input[-1]
    # input the main block
    state_input.append([input_id, u_move[0], u_move[1], u_move[2]])
    # input the supporting block
    if support_id == None:
        pass
    else: 
        state_input.append([support_id, x_support, y_support, theta_support])

    if block_type == "metalrectangle" or block_type == "metalrtriangle":
        metalblock = metalBlock(u_move[0],u_move[1],w_block,h_block,u_move[2])
        display.drawobject(metalblock)
    elif block_type == "woodrectangle" or block_type == "woodrtriangle":
        woodblock = woodBlock(u_move[0],u_move[1],w_block,h_block,u_move[2])
        print(u_move)
        display.drawobject(woodblock)
    if support_type == "metalrectangle" or support_type == "metalrtriangle":
        metalblock = metalBlock(x_support,y_support,w_support,h_support,theta_support)
        display.drawobject(metalblock)
    elif support_type == "woodrectangle" or support_type == "woodrtriangle":
        woodblock = woodBlock(x_support,y_support,w_support,h_support,theta_support)
        display.drawobject(woodblock)
    elif support_type == None:
        pass

    plt.show(block=False)
    plt.pause(12)
    #run_simulation(level_select, movable_ID, ID_dict, state_input)
    # observe again about the main trajectory   
    traj_list, trace_time, collision, n_obj, bool_success, ID_list, _, _ = logging_trajectory("bb", level_select)   
    mainblock_traj = traj_list[pick_main_idx]
    contact_idx = 0
    for collision_element in collision:
        # [time, collision type, objA, objB]
        if (collision_element[2] == input_id and collision_element[3] != id_ball) or (collision_element[3] == input_id and collision_element[2] != id_ball):
            if collision_element[1] == "collisionStart":
                contact_idx = 0
                while abs(collision_element[0] - trace_time[contact_idx]) > 10:
                    contact_idx = contact_idx+1
    
    # 6) Compute the required force and point (if wood)
    if block_type[0:5] == "metal":
        pass
    else:
        p2 = ComputeSupportForces(block_type, mainblock_desired_state, mainblock_traj, contact_idx)
        #desired_state(x,y,w,h,rot,vx,vy,w_rot)
    print(p2)
    # 7) Assign the supporting blocks    
    for pick_support_idx in range(num_movable):
        if pick_support_idx == pick_main_idx:
            pass
        else:

            # Pick supporting blocks / we can add some priorities    
            support_id = movable_ID[pick_support_idx]
            support_block_type = ID_dict[support_id] # 'metalrectangle'
            support_block_state = ID_state_matching[support_id] # [x,y,w,h,rot]
    
    
    # 8) Observe the ball's traj.
    # 9) Check the error between guide_path_local and mainblock_traj at x_local    
    # 10) Error bound is satisfied --> success / violated --> update
    # f_actual = [x,y,dir]

    #unew = UpdateModelAfterFailure(guide_path_local, direction_end, block_type, block_state, s_ball_ini, u_optimal, f_actual)











