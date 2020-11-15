'''
© 2020 Hotae Lee <hotae.lee@berkeley.edu>
'''

import numpy as np
from matplotlib import pyplot as plt
from math import pi, cos, sin

from functions.path_planning import path_planning, re_planning
import functions.get_keydirection as keydirection
from functions.physics.obj_class import Ball, metalBlock, woodBlock, powerUps
from functions.parsing_movableobjects_levels import parsing_objects, run_simulation, logging_trajectory
from functions.prediction import CheckCollisionWithFixed, ImpactMapBall2Fixed, ImpactMapBall2Wood
from functions.physics.common.const import g, dt
from functions.localregion import SortoutEnv, LocalRegion

'''
0. import the enviornment setting
'''
level = 21
id_grd, s_grd, s_total, id_total, n_total, movable_ID, ID_dict, ID_state_matching = parsing_objects(level)

type_obj_list = ["ground" for i in range(len(id_grd))]

# s_total = [s_ball, s_flag, s_metal, s_wood, s_woodtri, s_speedupu, s_speedupd, s_speedupl, s_speedupr, s_slowdown, s_gravity, s_gravitydown, s_spring, s_teleport]
# s_ball = [x,y,w,h,rot]
'''
1. path planning
'''
n_sample = 800
map_size = [0,500,0,700]
k1 = 6
k2 = 10
n_exclude = 1
prm, shortest_path, sampling_list, ax1 = path_planning(level, n_sample, k1 , map_size)    
#re_planning(prm, level, shortest_path, k2, n_exclude)
level_select = 21
state_input = []
#guide_path = [[25, 100], [63, 123], [98, 129], [119, 150], [138, 181], [155, 193], [175, 199], [216, 216], [257, 214], [291, 208], [338, 223], [366, 240], [381, 253], [406, 258], [434, 265]]
guide_path = shortest_path
# for level 5
#guide_path = [[210, 100], [241, 119], [284, 139], [304, 129], [348, 137], [372, 144], [391, 151], [435, 153], [476, 180], [446, 210], [407, 223], [389, 227], [377, 260], [345, 276], [310, 274], [276, 263], [255, 283], [243, 287], [209, 294], [176, 288], [159, 292], [139, 293], [117, 300], [86, 302], [59, 292], [34, 285]]
# for level 21
guide_path = [[35, 135], [100,125], [140,90], [181,100], [207,70], [250,76], [300,88], [323,95], [350,120], [371,162], [400, 175]]
# Initialize
id_grd, s_grd_list, s_total, id_total, n_total, movable_ID, ID_dict, ID_state_matching = parsing_objects(level_select)
state_input.append(["{}".format(movable_ID[0]), 0,0,0])
run_simulation(level_select, movable_ID, ID_dict, state_input)
state_input = []

# local region loop
data_pre = [0,0,0,0,0] # [x_sum, y_sum, xy_sum, xx_sum, N] for bounce
n_iter = 0
flag_success = 1
idx_local_start = 0
while flag_success != 0 and n_iter < 6:
    # if n_iter is too big, we need to update guide path again and reset idx_local_start = 0 again
    # start the whole process again with a new guide path
    n_iter += 1
    # p_start_update : actual state with min error, idx_loca_end : local guide path's end idx
    state_input_update, p_start_update, flag_success, data, idx_local_end = LocalRegion(guide_path, level_select, state_input, data_pre, idx_local_start)
    print(state_input_update, p_start_update, flag_success, data, idx_local_end)
    if flag_success == 0:
        # success
        print("We made success")
    elif flag_success == 1:
        # local success / go to next    
        #prm, shortest_path, _, _ = path_planning(level, n_sample, k1 , map_size, p_start_update)  
        #guide_path_new = guide_path[0:idx_local_end+1] + [path for path in shortest_path]
        print("guide_path in main py : {}".format(guide_path)) 
        state_input = state_input_update
        data_pre = data
        idx_local_start = idx_local_end-4
    else:
        print("Even though it fails, go to next local region")
        print("we need exploration")
        state_input = state_input_update
        data_pre = data
        idx_local_start = idx_local_end-4

#prm, guide_path, _, _ = path_planning(level_select, n_sample, k1 , map_size, p_start_update)  


'''
2. model prediction
'''

# def TrajPredict(ball, s_obs_fixed, type_obj_list, n_iter, n_timestep = 1):
#     # Iterative update
#     # s_obs_fixed : [x,y,w,h,rot] / s_obs_moving : [x,y,w,h,rot,vx,vy,wrot]
#     for i in range(n_iter):
#         # Check collision
#         collision, collision_angle = CheckCollisionWithFixed(ball, s_obs_fixed, type_obj_list, n_timestep)
#         print("{0} and pos: {1}, vel: {2}, angle: {3}".format(collision, [ball.x,ball.y], [ball.vx, ball.vy], collision_angle))
#         # Update
#         if not collision:       
#             ball.update([0,ball.m*g], n_timestep)   
#             ball.trajectory()   
#         else:
#             # suppose we get the first element
#             idx_collision = collision[0]
#             x_grd = s_obs_fixed[idx_collision][0]
#             y_grd = s_obs_fixed[idx_collision][1]
#             w_grd = s_obs_fixed[idx_collision][2]
#             h_grd = s_obs_fixed[idx_collision][3]
#             rot_grd = s_obs_fixed[idx_collision][4]/180*pi
#             f = ImpactMapBall2Fixed(ball, collision_angle)        
#             print("v1: {0}, force: {1}".format(ball.vy*cos(collision_angle)-ball.vx*sin(collision_angle),f))
#             ball.update([f[0],f[1]+ball.m*g])
#             ball.trajectory()   
            
#     traj = ball.trajectory()
#     xtraj = []
#     ytraj = []
#     for i in range(len(traj)):
#         xtraj.append(traj[i][0])
#         ytraj.append(traj[i][1])
#     return xtraj, ytraj

# # Initialize
# ball = Ball(s_total[0][0]+15,s_total[0][1]+15,0,0) # +15 means the center of the ball
# flag = s_total[1]
# xtraj, ytraj = TrajPredict(ball, s_grd, type_obj_list, 100, 3)
# plt.scatter(xtraj,ytraj, s = 2, c = 'green')
# #plt.show()

# # Get which blocks you can locate
# s_input = ID_state_matching[movable_ID[0]]
# print(ID_dict[movable_ID[0]])
# # Decide input [x,y,rot]
# s_input[0] = 260
# s_input[1] = 200
# s_input[4] = 0

# # Add an input
# s_obs = []
# for i in range(len(s_grd)):
#     s_obs.append(s_grd[i])
# s_obs.append(s_input)
# type_obj_list.append(ID_dict[movable_ID[0]])
# print(s_obs)
# ball = Ball(s_total[0][0]+15,s_total[0][1]+15,0,0)
# xtraj, ytraj = TrajPredict(ball, s_obs, type_obj_list, 300, 3)
# plt.scatter(xtraj,ytraj, s = 3, c = 'm')

# # Draw a new block 
# x = s_input[0]
# y = s_input[1]
# w = s_input[2]
# h = s_input[3]
# rot = s_input[4]
# if w == 75:
#     plt.plot([x,x+w,x,x],[y,y+h,y+h,y], c = 'y')
# else:
#     plt.plot([x,x+w,x+w,x,x],[y,y,y+h,y+h,y], c = 'y')

# plt.show()
# print(ytraj)
# print(type(xtraj))