import argparse
import time
import math
import msgpack
from enum import Enum, auto

import numpy as np
import matplotlib.pyplot as plt
import time

from planning_utils import *
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local, local_to_global

plt.rcParams["figure.figsize"] = [12, 12]

# Global variables needed to plot
grid = []
grid_goal = []
grid_start = []
grid_path = []
grid_pruned_path = []
grid_tolerance = 1.0

SAFETY_DISTANCE = 5
TARGET_ALTITUDE = 5



def plan_path(goal_LLA):
    global grid, grid_goal, grid_start, grid_path, grid_pruned_path, grid_tolerance
    
    print("Searching for a path ...")

    obstacle_file = 'colliders.csv'

    # TODO: read lat0, lon0 from colliders into floating point values
    lat0, lon0 = read_global_home(obstacle_file)

    # TODO: set home position to (lon0, lat0, 0)
    print('Set global home position from file ', lon0, lat0)
    home_position = (lon0, lat0, TARGET_ALTITUDE)

    # TODO: retrieve current global position
    start_LLA = home_position
    print('global current position {}'.format(start_LLA))
    
    # TODO: convert to current local position using global_to_local()
    local_start_NED = global_to_local(start_LLA, home_position)
    print('local_home NED ', local_start_NED)

    # Read in obstacle map
    data = np.loadtxt(obstacle_file, delimiter=',', dtype='Float64', skiprows=2)
        
    # Define a grid for a particular altitude and safety margin around obstacles
    grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
    print("North offset = {0}, East offset = {1}".format(north_offset, east_offset))
    # Define starting point on the grid (this is just grid center)
    # TODO: convert start position to current position rather than map center
    grid_start = (int(-north_offset + local_start_NED[0]), int(-east_offset + local_start_NED[1]))

    # Set goal as some arbitrary position on the grid
    # TODO: adapt to set goal as latitude / longitude position and convert
    
    local_goal_NED = global_to_local(goal_LLA, home_position)
     
    grid_goal = (int(-north_offset + local_goal_NED[0]), int(-east_offset + local_goal_NED[1]))
    print('local_goal_NED ', local_goal_NED, goal_LLA, ' grid goal ', grid_goal)

    # Run A* to find a path from start to goal
    # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
    # or move to a different search space such as a graph (not done here)
    print('Local Start and Goal: ', grid_start, grid_goal)
    path, path_cost = a_star(grid, heuristic, grid_start, grid_goal)
    grid_path = path
    # TODO: prune path to minimize number of waypoints
    # TODO (if you're feeling ambitious): Try a different approach altogether!
    #        pruned_path = prune_path_collinear(path, grid_tolerance)
    pruned_path = prune_path_bresenham(grid, path)
    grid_pruned_path = pruned_path
    print(pruned_path)
        
    # Convert path to waypoints
    waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]
    ow = waypoints[0]
    ang_waypoints = []
    ang_waypoints.append(ow)
    for i, w in enumerate(waypoints):
        if(i > 0):
            ang = math.atan2(w[0]-ow[0], w[1]-ow[1])
            ang_waypoints.append([w[0], w[1], w[2], ang])
            ow = w
             
    print('Waypoints ', ang_waypoints)


if __name__ == "__main__":

    goal_LLA = (-122.39343735,   37.79051296,    TARGET_ALTITUDE)
    plan_path(goal_LLA)
    # Stuff to draw the path plan on grid.
    plt.imshow(grid, origin='lower')
    # For the purposes of the visual the east coordinate lay along
    # the x-axis and the north coordinates long the y-axis.
    plt.plot(grid_start[1], grid_start[0], 'x')
    plt.plot(grid_goal[1], grid_goal[0], 'x')
    p = np.array(grid_path)
    plt.plot(p[:,1], p[:,0], 'r-')
    pp = np.array(grid_pruned_path)
    plt.plot(pp[:,1], pp[:,0], 'go')
    plt.xlabel('EAST')
    plt.ylabel('NORTH')
#    plt.title('A* path pruned with Collinearity')
    plt.title('A* path pruned with Bresenham')
    plt.grid()
    plt.show()

