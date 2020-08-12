'''
    @ Author : Hotae Lee
    @ Date : 08/11/2020
    @ Function : simple models for dynamic equations wiht a rough assumption 
    @ Parameters : 
    @ Variables:  
    @ Retruns :  
    @ Description : 
    @ TODO : 
'''
import numpy as np
from math import cos, sin, sqrt, pi

if __name__ == "__main__":
    from const import g, dt    
    from planning_algo.utils import RotatePts
else:
    from . const import g, dt
    from . planning_algo.utils import RotatePts

# A) Ball
# 1) ball to line (state: ball, input: l, theta, vx, vy, w)
def Ball2Line(ball, l, theta, vx, vy, w, v_thres = 0.01, w_thres = 0.01):
    vt = ball.vx*cos(theta)+ball.vy*sin(theta)
    v = vy*cos(theta)-vx*sin(theta)+l*w # v means the velocity of the contact point of the block 
    # rolling on the block
    if abs(v) <= v_thres and abs(w) <= w_thres:
        ball.x = ball.x+l*cos(theta)
        ball.y = ball.y+l*sin(theta)
        vend = sqrt(vt**2+2*g*cos(theta))
        ball.vx = vend*cos(theta)
        ball.vy = vend*sin(theta)
        status = "rolling"
    # hitting the ball
    else:
        ball.vx = v*sin(theta)+vt*cos(theta)
        ball.vy = v*cos(theta)-vt*sin(theta)
        status = "hitting"
    return ball

# 2) ball to circle (state: ball, input: x,y,v,w)
def Ball2Circle(ball, r, x, y, vx, vy, w, v_thres = 0.05):
    vt = ball.vx*(y-ball.y)/(ball.radius+r) - ball.vy*(x-ball.x)/(ball.radius+r)
    v = vy*(x-ball.x)/(ball.radius+r)-vx*(y-ball.y)/(ball.radius+r)
    sin_theta = (y-ball.y)/(ball.radius+r)
    cos_theta = (x-ball.x)/(ball.radius+r)
    if abs(v) <= v_thres:
        ball.vx = vt*sin_theta # vt*sin(theta)
        ball.vy = vt*cos_theta
    else:       
        ball.vx = vt*sin_theta-v*cos_theta # vt*sin(theta)
        ball.vy = vt*cos_theta-v*sin_theta
    return ball

# 3) ball to point
def Ball2Point(ball, x, y):
    sin_theta = (y-ball.y)/ball.radius
    cos_theta = (x-ball.x)/ball.radius
    vt = ball.vy*sin_theta-ball.vx*cos_theta
    ball.vx = vt*cos_theta
    ball.vy = -vt*sin_theta
    return ball

# 4) ball in the air
def BallinAir(ball, l):
    ball.x += l
    ball.y += ball.vy/ball.vx*l+g/2*l**2/ball.vx**2
    ball.vy += g*l/ball.vx
    return ball

# B) Blocks
# 1) 1 point to line (pivot point : px, py)
def Point2Line(px,py,block1,block2):
    xcm = block1.x+block1.w/2
    ycm = block1.y+block1.h/2
    theta = block2.rot
    d_left = block1.h # need to check
    d_right = block1.w
    if xcm>px: 
        # falling to the right
        block1.x = px + d_right*cos(theta)/2 - d_left*sin(theta)/2 - d_right/2
        block1.y = py - d_right*sin(theta)/2 - d_left*cos(theta)/2 - d_left/2
        block1.rot = block2.rot
    else:
        # falling to the left
        block1.x = px - d_left*cos(theta)/2 - d_right*sin(theta)/2 - d_right/2
        block1.y = py - d_right*cos(theta)/2 + d_left*sin(theta)/2 - d_left/2
        block1.rot = block2.rot + 90

    if abs(block2.vx) < 0.05 and abs(block2.vy) < 0.05 and abs(block2.w_rot) < 1:
        # The upper block is stopped while attching to the lower block.
    else:
        # angular momentum conservation
        m1 = block1.den*block1.w*block1.h
        m2 = block2.den*blcok2.w*block2.h
        I1 = m1/12*(block1.w**2+block1.h**2)
        I2 = m2/12*(block2.w**2+block2.h**2)
        vnew_x = (m1*block1.vx + m2*block2.vx)/(m1+m2)
        vnew_y = (m1*block1.vy + m2*block2.vy)/(m1+m2)
        l1 = 0
        l2 = 0
        wnew = (I1*block1.w_rot+I2*blcok2.w_rot)/(I1+I2+m1*l1**2+m2*l2**2)
        block1.vx = vnew_x
        block2.vx = vnew_x
        block1.vy = vnew_y
        block2.vy = vnew_y
        block1.w_rot = wnew
        block2.w_rot = wnew
# 2) 2pts to line (pivot point : px1, py1, px2, py2)
def Points2Line(px1,py1,px2,py2,block):
    xcm = block.x+block.w/2
    ycm = block.y+block.h/2
    if (px1-xcm)*(px2-xcm) > 0:
        # the point close to C.M. becomes the pivot point.
        if abs(px1-xcm) < abs(px2-xcm):
            # p1 can be a pivot
        else:
            # p2 can be a pviot
    else:
        # static for the most cases

# 3) line to circle


if __name__ == "__main__":
    pts = RotatePts([0,0,10,20,90])
    pts1 = RotatePts([0,0,10,20,-90])
    print(pts)
    print(pts1)
    print(type(pts))