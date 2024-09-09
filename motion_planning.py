import argparse
import time
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
grid_g = []
grid_goal_g = []
grid_start_g = []
grid_path_g = []
grid_pruned_path_g = []
grid_tolerance_g = 1.0

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)
        

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        obstacle_file = 'colliders.csv'

        # TODO: read lat0, lon0 from colliders into floating point values
        lat0, lon0 = read_global_home(obstacle_file)

        # TODO: set home position to (lon0, lat0, 0)
        print('Set global home position from file ', lon0, lat0)
        self.set_home_position(lon0, lat0, 0.0)

        # TODO: retrieve current global position
        print('global current position {}'.format(self.global_position))

        # TODO: convert to current local position using global_to_local()
        local_home_NED = global_to_local(self.global_position, self.global_home)
        print('local_home NED ', local_home_NED)
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position, self.local_position))

        # Read in obstacle map
        data = np.loadtxt(obstacle_file, delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, East offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        # TODO: convert start position to current position rather than map center
        grid_start = (int(-north_offset + local_home_NED[0]), int(-east_offset + local_home_NED[1]))

        # Set goal as some arbitrary position on the grid
        # TODO: adapt to set goal as latitude / longitude position and convert
        goal_lat_lon_alt = (-122.39343735,   37.79051296,    TARGET_ALTITUDE)
 
        local_goal = global_to_local(goal_lat_lon_alt, self.global_home)

        grid_goal = (int(-north_offset + local_goal[0]), int(-east_offset + local_goal[1]))
        print('local_goal NED ', local_goal, goal_lat_lon_alt, ' grid goal ', grid_goal)

        global grid_g
        global grid_goal_g
        global grid_start_g
        global grid_tolerance_g
        grid_g = grid
        grid_goal_g = grid_goal
        grid_start_g = grid_start

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, path_cost = a_star(grid, heuristic, grid_start, grid_goal)
        print('Path ', path, path_cost)

        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!
#        pruned_path = prune_path_collinear(path, grid_tolerance_g)
        pruned_path = prune_path_bresenham(grid, path)
        global grid_path_g
        grid_path_g = path
        global grid_pruned_path_g
        grid_pruned_path_g = pruned_path
        
        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()

    print(grid_g, grid_start_g, grid_goal_g)
    plt.imshow(grid_g, origin='lower')
    # For the purposes of the visual the east coordinate lay along
    # the x-axis and the north coordinates long the y-axis.
    plt.plot(grid_start_g[1], grid_start_g[0], 'x')
    plt.plot(grid_goal_g[1], grid_goal_g[0], 'x')
    p = np.array(grid_path_g)
    plt.plot(p[:,1], p[:,0], 'r-')
    pp = np.array(grid_pruned_path_g)
    plt.plot(pp[:,1], pp[:,0], 'go')
    plt.xlabel('EAST')
    plt.ylabel('NORTH')
#    plt.title('A* path pruned with Collinearity')
    plt.title('A* path pruned with Bresenham')
    plt.grid()
    plt.show()

