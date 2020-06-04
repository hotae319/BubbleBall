# RPM Class
import random
from math import sqrt, cos, sin, pi, atan
from utils import CheckInside, CheckIntersect, CheckIntersectPolygon, ClosestNeighbor

class PRM:
    def __init__(self, map_size, s_obs, p_start, p_end):
        self.map_size = map_size # [xmin=0, xmax, ymin=80, ymax]        
        self.obs_slist = obs_slist # [x,y,width,height,rotation] N by 5
        self.p_start = p_start # [x,y]
        self.p_end = p_end
        self.path = [p_start]
        self.p_sample = []
    def ObstacleEnv(self, ptest = [0,0]):
        # Check if ptest is inside the obstacle region
        # Assume the rectangle
        free_bool = True
        obs_region = []
        for s_obs in self.obs_slist:
            w = s_obs[2]
            h = s_obs[3]
            theta = s_obs[4]/180*pi
            alpha = atan(h/w) # we can replace it with using cos(a+b) = cos(a)cos(b)-sin(a)sin(b)
            cx = s_obs + w/2
            cy = s_obs + h/2
            l = sqrt(w**2+h**2)/2
            obs_region.append([[cx+l*cos(theta+alpha),cy+l*sin(theta+alpha)],[cx+l*cos(theta-alpha),cy+l*sin(theta-alpha)],
            [cx-l*cos(theta+alpha),cy-l*sin(theta+alpha)],[cx-l*cos(theta-alpha),cy-l*sin(theta-alpha)]]) # [p1,p2,p3,p4] = [[x,y],p2,p3,p4], clockwise
        for obs_polygon in obs_region:
            if CheckInside(obs_polygon,ptest):
                free_bool = False
        return free_bool, obs_region
    def Sampling(self, num_samples):
        '''
        1) connect p_start and p_end 2) select the samples around that line (average is the line) 
        '''
        xmin = 0
        ymin = 80
        xmax = self.map_size[1]
        ymax = self.map_size[3]        
        num = 0
        while num>=num_samples:
            # sampling randomly
            xtemp = random.randint(xmin, xmax)
            ytemp = random.randint(ymin, ymax)
            # sampling around the line + sampling randomly
            free_bool, obs_region = ObstacleEnv([xtemp,ytemp])
            if free_bool:
                self.p_sample.append([xtemp,ytemp])
                num += 1
        return self.p_sample
    def AddStartEnd(self):
    def ConnectDots(self, k):        
        n_smp = len(self.p_sample)
        connect_line = []
        free_bool, obs_region = ObstacleEnv()
        # k closest neighbors
        for pt in Sampling(n_smp):
            knn = ClosestNeighbor(self.p_sample,pt,k)
            for q in knn:
                bool_linefree = True
                for obs_polygon in obs_region:
                    if CheckIntersectPolygon(obs_polygon,pt,q):
                        bool_linefree = False      
                if bool_linefree:
                    connect_line.append([pt,q]) # connect_line = [[p1,q1],[p2,q2],...] = [[[1,2],[2,3]],[[2,5],[3,3]],...]
        return connect_line

    def FindShortestPath(self):

