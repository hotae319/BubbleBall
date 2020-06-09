import os, sys
import numpy as np

# add the absolute path of /parsing_bubbleball
#sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
if __name__ == "__main__":
    from log2matrix import logging_trajectory
    from utils import FindParabola
else:
    from . log2matrix import logging_trajectory
    from . utils import FindParabola

def GetKeyDirection(waypoints):
	'''
	Args : waypoints(shortest_path)
	Returns : key_direction, parabola
	'''
	