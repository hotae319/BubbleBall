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
from scipy.optimize import minimize, Bounds, LinearConstraint, NonlinearConstraint
from math import cos, sin, sqrt, pi

if __name__ == "__main__":
    from common.const import g, dt    
    #from planning_algo.utils import RotatePts    
else:
    from . common.const import g, dt
    #from . planning_algo.utils import RotatePts

# A) Ball
# 1) ball to line (state: ball, input: l, theta, vx, vy, w)
def Ball2Line(ball, l, theta, vx, vy, w, v_thres = 0.01, w_thres = 0.01):
    vt = ball.vx*cos(theta)+ball.vy*sin(theta)
    v = vy*cos(theta)-vx*sin(theta)+l*w # v means the velocity of the contact point of the block 
    # rolling on the block
    if abs(v) <= v_thres and abs(w) <= w_tshres:
        ball.x = ball.x+l*cos(theta)
        ball.y = ball.y+l*sin(theta)
        vend = sqrt(vt**2+2*g*sin(theta))
        ball.vx = vend*cos(theta)
        ball.vy = vend*sin(theta)
        status = "rolling"
    # hitting the ball
    else:
        ball.vx = v*sin(theta)+vt*cos(theta)
        ball.vy = v*cos(theta)-vt*sin(theta)
        status = "hitting"
    return ball, status

def Ball2LineValue(xball, yball, vxball, vyball, rball, l, theta, vx, vy, w, v_thres = 0.01, w_thres = 0.01):
    # degree to radians 
    theta = theta/180*pi
    vt = vxball*cos(theta)+vyball*sin(theta)
    v = vy*cos(theta)-vx*sin(theta)+l*w # v means the velocity of the contact point of the block 
    # rolling on the block
    if abs(v) <= v_thres and abs(w) <= w_thres:
        xball = xball+l*cos(theta)
        yball = yball+l*sin(theta)
        vend = sqrt(vt**2+2*g*sin(theta))
        vxball = vend*cos(theta)
        vyball = vend*sin(theta)
        status = "rolling"
    # hitting the ball
    else:
        vxball = v*sin(theta)+vt*cos(theta)
        vyball = v*cos(theta)-vt*sin(theta)
        status = "hitting"
    ball = [xball,yball,vxball,vyball]
    return ball, status

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

def Ball2CircleValue(xball, yball, vxball, vyball, rball, r, x, y, vx, vy, w, v_thres = 0.05):
    vt = vxball*(y-yball)/(rball+r) - vyball*(x-xball)/(rball+r)
    v = vy*(x-xball)/(rball+r)-vx*(y-yball)/(rball+r)
    sin_theta = (y-yball)/(rball+r)
    cos_theta = (x-xball)/(rball+r)
    if abs(v) <= v_thres:
        vxball = vt*sin_theta # vt*sin(theta)
        vyball = vt*cos_theta
    else:       
        vxball = vt*sin_theta-v*cos_theta # vt*sin(theta)
        vyball = vt*cos_theta-v*sin_theta
    ball = [xball,yball,vxball,vyball]
    return ball

# 3) ball to point
def Ball2Point(ball, x, y):
    sin_theta = (y-ball.y)/ball.radius
    cos_theta = (x-ball.x)/ball.radius
    vt = ball.vy*sin_theta-ball.vx*cos_theta
    ball.vx = vt*cos_theta
    ball.vy = -vt*sin_theta
    return ball

def Ball2PointValue(xball, yball, vxball, vyball, rball, x, y):
    sin_theta = (y-yball)/rball
    cos_theta = (x-xball)/rball
    vt = vyball*sin_theta-vxball*cos_theta
    vxball = vt*cos_theta
    vyball = -vt*sin_theta
    ball = [xball,yball,vxball,vyball]
    return ball

# 4) ball in the air
def BallinAir(ball, l):
    ball.x += l
    ball.y += ball.vy/ball.vx*l+g/2*l**2/ball.vx**2
    ball.vy += g*l/ball.vx
    return ball

def BallinAirValue(xball, yball, vxball, vyball, lx, ly):
    # l denotes the distance along x axis, l>0
    if lx == 0:
        yball += ly
        vyball += sqrt(vyball**2+2*g*ly)
    else:
        xball += lx
        yball += vyball/vxball*lx+g/2*lx**2/vxball**2
        vyball += g*lx/vxball
    ball = [xball,yball,vxball,vyball]
    return ball


# 5) ball in the small region (combine air and ground)
def BallinEnvValue(xball, yball, vxball, vyball, rball, x, y, w, h, theta, xregion, yregion, xsize, ysize):
    # Start from right or left
    if xball-xregion > xregion+xsize-xball:
        #From right
        x0 = xball+xsize
        x3 = xball
    else:
        #From left
        x0 = xball
        x3 = xball+xsize
    # Find the boundary
    theta = theta/180*pi
    if x+w/2-w/2*cos(theta)+h/2*sin(theta) < x0:
        xleft = x0
    else:
        xleft = x+w/2-w/2*cos(theta)+h/2*sin(theta)        
    if x1+w*cos(theta) > x0+xsize:
        xright = x0+xsize
    else:
        xright = x1+w*cos(theta)
    #y1 = y+h/2-w/2*sin(theta)-h/2*cos(theta)    
    # Decide x1, x2
    if x0 == xball:
        x1 = xleft
        x2 = xright
    else:
        x1 = xright
        x2 = xleft
    # Check collisions
    ball = BallinAirValue(xball, yball, vxball, vyball, lx, ly)


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
        status = "attched"
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
        status = "moving"
    return block1, block2, status
# 2) 2pts to line (pivot point : px1, py1, px2, py2)
def Points2Line(px1,py1,px2,py2,block, block1, block2):
    # block1 which contacts the main block at p1
    # block2 which contacts the main block at p2
    xcm = block.x+block.w/2
    ycm = block.y+block.h/2
    if (px1-xcm)*(px2-xcm) > 0:
        # the point close to C.M. becomes the pivot point.
        if abs(px1-xcm) < abs(px2-xcm):
            # p1 can be a pivot
            block, block1 = Point2Line(px1, py1, block, block1)
        else:
            # p2 can be a pviot
            block, block2 = Point2Line(px1, py1, block, block2)            
    return block, block1, block2
# 3) line to circle
def Line2Circle(block1, block2, px, py, l):
    # block1 should be a line, upper block
    # l : the distance along the side(the direction of the block)
    xcm1 = block1.x+block1.w/2
    ycm1 = block1.y+block1.h/2    
    xcm2 = block2.x+block2.w/2
    ycm2 = block2.y+block2.h/2   
    d_left = block1.h # need to check
    d_right = block1.w
    vt = vx*cos(block1.rot)+vy*sin(block1.rot)
    if xcm1 > xcm2:
        # move to the right
        block1.x += l*cos(block1.rot)
        block1.y += l*sin(block1.rot)
        vend = sqrt(2*g*sin(block1.rot)+vt**2)
        block1.vx = vend*cos(block1.rot)
        block1.vy = vend*sin(block1.rot)
    else:
        # move to the left
        block1.x -= l*cos(block1.rot)
        block1.y -= l*sin(block1.rot)
        vend = sqrt(2*g*sin(block1.rot)+vt**2)
        block1.vx = -vend*cos(block1.rot)
        block1.vy = -vend*sin(block1.rot)
    return block1
# 4) Circle to line 
def Circle2Line(block1, block2, l, vx, vy, w):
    # block1 should be a circle
    theta = block2.rot
    vt = block1.vx*cos(theta)+block1.vy*sin(theta)
    v = vy*cos(theta)-vx*sin(theta)+l*w # v means the velocity of the contact point of the block 
    # rolling on the block
    block1.x = block1.x+l*cos(theta)
    block1.y = block1.y+l*sin(theta)
    vend = sqrt(vt**2+2*g/3*cos(theta)) # reduced acceleration
    block1.vx = vend*cos(theta)
    block1.vy = vend*sin(theta)
    block1.w_rot = vend/block1.radius
    return block1

def f(x,*y):
    # x[0] = l, x[1] = theta, x[2] = vx, x[3] = vy, x[4] = w
    # y[0] = ball.x, y[1] = ball.y, y[2] = ball.vx, y[3] = ball.vy
    state_ball, _ = Ball2LineValue(y[0],y[1],y[2],y[3],x[0],x[1],x[2],x[3],x[4])
    error = (state_ball[0]-100)**2+(state_ball[1]-100)**2+0*state_ball[2]**2+0*state_ball[3]**2 
    return error


if __name__ == "__main__":
    #ball, status = Ball2Line(ball, l, theta, vx, vy, w, v_thres = 0.01, w_thres = 0.01)
    x0 = [10,0,0,0,0]
    y = (50,50,0,15)
    search_bound = Bounds([0,-90,-np.inf,-np.inf,-np.inf],[np.inf,90,np.inf,np.inf,np.inf])
    f_nonlin = lambda x:x[0]*x[1]
    nonlin_constr = NonlinearConstraint(f_nonlin,0,np.inf)
    res = minimize(f,x0, args = y, bounds = search_bound, constraints = nonlin_constr)
    print(res.x, res.fun)
    import os, sys

    abspath = os.path.dirname(os.path.abspath(os.path.dirname(__file__)))
    print(abspath)