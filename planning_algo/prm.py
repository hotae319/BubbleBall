# RPM Class
import random
from math import sqrt, cos, sin, pi, atan
from utils import CheckInside, CheckIntersect, CheckIntersectPolygon, ClosestNeighbor

class PRM:
    def __init__(self, map_size, obs_slist, p_start, p_end):
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
        if type(self.obs_slist[0]) == list:
            for s_obs in self.obs_slist:
                w = s_obs[2]
                h = s_obs[3]
                theta = s_obs[4]/180*pi
                alpha = atan(h/w) # we can replace it with using cos(a+b) = cos(a)cos(b)-sin(a)sin(b)
                cx = s_obs[0] + w/2
                cy = s_obs[1] + h/2
                l = sqrt(w**2+h**2)/2
                obs_region.append([[cx+l*cos(theta+alpha),cy+l*sin(theta+alpha)],[cx+l*cos(theta-alpha),cy+l*sin(theta-alpha)],
                [cx-l*cos(theta+alpha),cy-l*sin(theta+alpha)],[cx-l*cos(theta-alpha),cy-l*sin(theta-alpha)]]) # [p1,p2,p3,p4] = [[x,y],p2,p3,p4], clockwise
        else: # only 1 obstacle
            s_obs = self.obs_slist
            w = s_obs[2]
            h = s_obs[3]
            theta = s_obs[4]/180*pi
            alpha = atan(h/w) # we can replace it with using cos(a+b) = cos(a)cos(b)-sin(a)sin(b)
            cx = s_obs[0] + w/2
            cy = s_obs[1] + h/2
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
        ymin = 0
        xmax = self.map_size[1]
        ymax = self.map_size[3]        
        num = 0
        while num<=num_samples:
            # sampling randomly
            xtemp = random.randint(xmin, xmax)
            ytemp = random.randint(ymin, ymax)
            # sampling around the line + sampling randomly
            free_bool, obs_region = self.ObstacleEnv([xtemp,ytemp])
            if free_bool and not ([xtemp,ytemp] in self.p_sample):
                self.p_sample.append([xtemp,ytemp])
                num += 1
        return self.p_sample  
    def Addstartend(self):
        # Add start/end point
        self.p_sample.append(self.p_start)
        self.p_sample.append(self.p_end)
    def ConnectDots(self, k):        
        n_smp = len(self.p_sample)
        connect_line = []
        free_bool, obs_region = self.ObstacleEnv()
        # k closest neighbors
        for pt in self.Sampling(n_smp):
            knn = ClosestNeighbor(self.p_sample,pt,k)
            for q in knn:
                bool_linefree = True
                for obs_polygon in obs_region:
                    if CheckIntersectPolygon(obs_polygon,pt,q):
                        bool_linefree = False      
                if bool_linefree and not ([q,pt] in connect_line):
                    connect_line.append([pt,q]) # connect_line = [[p1,q1],[p2,q2],...] = [[[1,2],[2,3]],[[2,5],[3,3]],...]
        return connect_line
    def FindConnectedNeighbor(self, connect_line, pt):
        neighbors = []
        for line in connect_line:
            if pt == line[0]:
                neighbors.append(line[1])
            elif pt == line[1]:
                neighbors.append(line[0])
        return neighbors
    def Astar(self, connect_line):
        frontier = []
        priority = []
        cost_so_far = {}
        came_from = {}
        path = []
        # start 
        frontier.append(self.p_start)     
        priority.append(0)   
        cost_so_far["{}".format(self.p_start)] = 0
        came_from["{}".format(self.p_start)] = None

        while (not frontier) == False:
            # Choose the highest priority of frontier list as the current
            current = frontier[0]
            frontier.remove(current)
            priority.remove(priority[0])
            # Find the path
            if current == self.p_end:
                break
            for nexts in self.FindConnectedNeighbor(connect_line,current):
                new_cost = cost_so_far["{}".format(current)] + sqrt((current[0]-nexts[0])**2+(current[1]-nexts[1])**2)
                # Add next into the frontier or change cost
                if nexts not in frontier or new_cost < cost_so_far["{}".format(nexts)]:
                    cost_so_far["{}".format(nexts)] = new_cost 
                    cost_p = new_cost + abs(self.p_end[0]-nexts[0])+abs(self.p_end[1]-nexts[1])
                    # Check next's priority
                    i = 0
                    for cost_temp in priority:
                        if cost_temp >= cost_p:
                            i = priority.index(cost_temp)
                            break
                        elif priority.index(cost_temp) == len(priority)-1:
                            i = len(priority)
                    frontier.insert(i,nexts)
                    priority.insert(i,cost_p)
                    came_from["{}".format(nexts)] = current
            path.append(current)
        return path, came_from

# Example about How to use
if __name__ == "__main__":
    map_size = [0,10,0,10]
    obs_list = [2,2,3,3,0]
    p_start = [0,0]
    p_end = [9,9]
    prm = PRM(map_size, obs_list, p_start, p_end)
    num_samples = 30
    sampling_list = prm.Sampling(num_samples)
    prm.Addstartend()
    k = 3
    connect_line = prm.ConnectDots(k)
    shortest_path, came_from = prm.Astar(connect_line)
    print(shortest_path)