# Class
from math import pi
if __name__ == "__main__":
    from const import g, dt    
else:
    from . const import g, dt    
class Ball:
    def __init__(self, x, y, vx, vy):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.radius = 15
        self.den = 0.2
        self.e = 0.4
        self.m = 4/3*pi*(self.radius)**3*self.den
        self.traj = [[self.x, self.y, self.vx, self.vy]]
    def update(self, force, n_timestep = 1):
        # force = [Fx, Fy]
        dt_new = dt*n_timestep 
        self.vx += force[0]/self.m*dt_new
        self.vy += force[1]/self.m*dt_new
        self.x += self.vx*dt_new-0*force[0]/self.m*dt_new*dt_new
        self.y += self.vy*dt_new-0*force[1]/self.m*dt_new*dt_new
        new_state = [self.x, self.y, self.vx, self.vy]
        return new_state
    def updateValue(self, force, n_timestep):
        # force = [Fx, Fy]
        dt_new = dt*n_timestep 
        vx_new = self.vx + force[0]/self.m*dt_new
        vy_new = self.vy + force[1]/self.m*dt_new
        x_new = self.x + self.vx*dt_new + force[0]/self.m*dt_new*dt_new
        y_new = self.y + self.vy*dt_new + force[1]/self.m*dt_new*dt_new
        new_state = [x_new, y_new, vx_new, vy_new]
        return new_state
    def trajectory(self):
        self.traj.append([self.x, self.y, self.vx, self.vy])
        return self.traj

class metalBlock:
    # It can also cover the ground piece (same friction)
    def __init__(self, x, y, w, h, rot):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.rot = rot
        self.fric = 0.5

class woodBlock:
    def __init__(self, x, y, w, h, rot, vx, vy, w_rot, cir_tri):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.rot = rot
        self.vx = vx
        self.vy = vy
        self.w_rot = w_rot
        self.fric = 0.5
        self.den = 1.5 # catapult 0.5
        if w == 50 or w == 100 or w == 150:
            self.m = self.den*w*h
        elif w == 25:
            self.m = 4/3*pi*(w/2)**3*self.den
        elif cir_tri == True:
            self.m = 4/3*pi*(w/2)**3*self.den
        else:
            self.m = 0.5*w*h*self.den
        self.traj = []
    def update(self, force):
        # force = [Fx, Fy]
        self.vx += force[0]/self.m*dt
        self.vy += force[1]/self.m*dt
        self.x += self.vx*dt
        self.y += self.vy*dt
    def trajectory(self):
        self.traj.append([self.x, self.y, self.vx, self.vy])
        return self.traj

class powerUps:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.size = 26