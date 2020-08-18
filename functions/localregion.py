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
from scipy.optimize import minimize, Bounds, LinearConstraint, NonlinearConstraint
import random
from math import cos, sin, pi, sqrt, atan
import os, sys
if __name__ == "__main__":
    #from obj_class import Ball, metalBlock, woodBlock, powerUps
    from physics.simple_models import Ball2LineValue, Ball2CircleValue, BallinAirValue
    from physics.common.const import g, dt
else:
    #from . obj_class import Ball, metalBlock, woodBlock, powerUps
    from . physics.simple_models import Ball2LineValue, Ball2CircleValue, BallinAirValue
    from . physics.common.const import g, dt


def SelectLocalRegion(guide_path, x_predicted_traj, y_predicted_traj, error_threshold = 0.1):
    # predicted traj : xtraj = [1,2,...], ytraj = [2,3,...]
    # guide_path(shortest_path): [[1,2][2,3]...]
    num_pred_traj = len(predicted_traj)
    num_guide_path = len(guide_path)
    dist_list = []
    error_list = []
    sum_dist = 0    
    x_predicted_traj_new = []
    y_predicted_traj_new = []
    for i in range(num_guide_path):
        xdist = guide_path[i][0]-guide_path[i+1][0]
        ydist = guide_path[i][1]-guide_path[i+1][1]
        dist = sqrt(xdist**2+ydist**2)
        sum_dist += dist
        dist_list.append(sum_dist)      
    # pick the pts of predicted traj to compute the tracking error
    # The entire predicted traj. is split into several parts which are proportional to interval of Tg
    for i in range(num_guide_path):
        id_pick = int(num_pred_traj*dist_list[i]/sum_dist)
        x_predicted_traj_new.append(x_predicted_traj[id_pick])
        y_predicted_traj_new.append(y_predicted_traj[id_pick])
        error_list.append(sqrt((guide_path[i][0]-x_predicted_traj_new[i])**2+(guide_path[i][1]-y_predicted_traj_new[i])**2))
    idx_local = 0
    while error_list[idx_local] < error_threshold:
        idx_local += 1
    # Decide the local region
    # id_pick : predicted traj's idx
    id_pick = int(num_pred_traj*dist_list[idx_local]/sum_dist)
    guide_path_local = guide_path[0:idx_local+1] 
    # direction of the end element of local guide path, angle of atan(y/x)
    direction_end = atan((guide_path[idx_local+1][1]-guide_path[idx_local][1])/(guide_path[idx_local+1][0]-guide_path[idx_local][0]))
    # It can be changed to the total prediction traj
    x_pred_max = max(x_predicted_traj_new)
    y_pred_max = max(y_predicted_traj_new)
    x_guide_max = guide_path_local[idx_local][0]
    y_guide_max = guide_path_local[idx_local][1]
    x_local = max(x_pred_max,x_guide_max)
    y_local = max(y_pred_max,y_guide_max)
    return guide_path_local, direction_end, x_local, y_local

def SortoutEnv(x_local, y_local, s_grd):
    # ground type is only rectangle, so we don't need ID or type here
    num_grd = len(s_grd)
    env_local = []
    for i in range(num_grd):
        if s_grd[i][0] <= x_local and s_grd[i][1]<=y_local:
            env_local.append(s_grd[i])
    return env_local

def PickMainBlock(movable_ID, ID_dict, guide_path_local, env_local):
    # for now they just randomly pick one block as a main block
    # We can update this with physics or classification network
    num_movable = len(movable_ID)
    random.shuffle(movable_ID)
    order2pick = movable_ID
    return order2pick

def fobj(x,*y):
    # Common things irrelevant to the type of block
    # y[0] = ball.x, y[1] = ball.y, y[2] = ball.vx, y[3] = ball.vy, y[4] = ball.r
    # y[5] = guide_path_local[end][0] : x of guide path, y[6] : y of guide path
    # y[7] : direction of v, vx/vy of guide path
    # y[8] : block type, y[9] : block state [x,y,w,h,rot]

    # tracking error (ref = [x,y,dir])
    ref = [y[5],y[6],y[7]]  

    # All rollouts consist of "gravity or environment -> input -> gravity or environment"
    if y[8] == "rectangle":
        # 1) retangle
        # x[0] = l, x[1] = theta, x[2] = vx, x[3] = vy, x[4] = w
        #  l should be lower than y[9][3] = w 
        state_ball, _ = Ball2LineValue(y[0],y[1],y[2],y[3],y[4],x[0],x[1],x[2],x[3],x[4])
    elif y[8] == "circle":
        # 2) circle
        # x[0] = x, x[1] = y, x[2] = vx, x[3] = vy, x[4] = w
        # r is given as y[9][2]/2
        state_ball_collision = Ball2CircleValue(y[0],y[1],y[2],y[3],y[4],y[9][2]/2,x[0],x[1],x[2],x[3],x[4])
        x_distance = abs(ref[0]-y[0])
        state_ball = BallinAirValue(state_ball_collision[0],state_ball_collision[1],state_ball_collision[2],state_ball_collision[3],x_distance)    
    if abs(state_ball[2]) <0.01: # if ball.vx is too low
        direction = pi/2
    else:
        direction = atan(state_ball[3]/state_ball[2])    
    error = (state_ball[0]-ref[0])**2+(state_ball[1]-ref[1])**2+(direction-ref[2])**2    

    return error


def FindOptimalInput(guide_path_local, direction_end, block_type, block_state, s_ball_ini):
    # the size of the local region and the environment in the local region can be input
    # But, we want to minimize the intervention of environment.
    # Input : guide_path_local, block_type, block_state[x,y,w,h,rot], s_ball_ini (x,y,vx,vy)
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
    if block_type == "rectangle":
        # x[0] = l, x[1] = theta, x[2] = vx, x[3] = vy, x[4] = w
        x0 = [10,0,0,0,0]        
        # bound : 0 < l < width, -90<theta<90
        search_bound = Bounds([0,-90,-np.inf,-np.inf,-np.inf],[block_state[2],90,np.inf,np.inf,np.inf])
        # nonlinear constr : l and theta have same sign
        f_nonlin = lambda x:x[0]*x[1]
        nonlin_constr = NonlinearConstraint(f_nonlin,0,np.inf)
        # solve optimization
        res = minimize(fobj ,x0, args = y, bounds = search_bound, constraints = nonlin_constr)
        print(res)
        u_input = res.x
    elif block_type == "circle":
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

def ComputeSupportForces(mainblock_type, mainblock_desired_state, main_traj, contact_point, contact_idx):
    # Compute the 2 forces or 1 force with a fixed force, given the desired state of a main block
    # Input : type("rectangle"), desired_state(x,y,w,h,rot,vx,vy,w_rot)
    #         current trajectory : main_traj([[vx,vy,w,x,y,rot],[],...,[]])
    #         contact_point : [x,y], contact_idx : index of traj. when the contact starts
    # Output : two points we will support (p1[x1,y1], p2[x2,y2]) / one point or range corresponding to given contact pt
    # the supporting forces which can compensate the movement
    if mainblock_type == "rectangle":
        m = 1.5*mainblock_desired_state[2]*mainblock_desired_state[3]
        I = 1/12*m*(mainblock_desired_state[2]**2+mainblock_desired_state[3]**2)
    elif mainblock_type == "circle":
        m = 1.5*pi*(mainblock_desired_state[3]/2)**2
        I = 1/2*m*(mainblock_desired_state[3]/2)**2
    elif mainblock_type == "triangle":
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
    lx = torque2/fy2
    x2 = mainblock_desired_state[0] + mainblock_desired_state[2]/2 + lx
    y2 = mainblock_desired_state[1] + mainblock_desired_state[3]/2 
    # we need the function to find the intersection 
    p2 = [x2,y2]
    return p2

def UpdateModelAfterFailure():
    model_paramter = 0
    return model_paramter
def UpdateCostWeights():
    weights = [0,0,1]
    return weights

if __name__=="__main__":
    #u = FindOptimalInput([[60,80],[100,100]], 0.3, "rectangle", [0,0,50,150,0],[60,80,0,5,15])
    u = FindOptimalInput([[60,80],[100,100]], 0.3, "circle", [0,0,50,50,0],[60,80,0,5,15])
    print(type(u))







