import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


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
                 # Difference                   
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
        #print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        
        #print('target position', self.target_position)
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

    def collinearity_int(self,p1, p2, p3, epsilon): 
        collinear = False
        det = p1[0]*(p2[1] - p3[1]) + p2[0]*(p3[1] - p1[1]) + p3[0]*(p1[1] - p2[1])
        print(det)
        if abs(det) <= epsilon:
            collinear = True
        return collinear

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("\n\n")
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # for some reason, it is setting the global home to my current position.  but it only does this once.  perhaps this is the intial position
        # of the drone.  Then why is the grid start such a large numbber?
        # It appears that the lat long in colliders.csv is exactly the global position of the drone when it starts up.
        # grid_start should be the first waypoint which is why it always returns to it first

        # Done: read lat0, lon0 from colliders into floating point values
        f = open('colliders.csv')
        line = f.readline()
        lat0, lon0 = float(line[5:14]), float(line[21:])
        print("lat {0}   lon {1}".format(lat0, lon0))
        
        # Done: set home position to (lon0, lat0, 0)
        # Setting the home position must be the way we align the grid with the simulator space
        # Then, when we send it to a local position, were are sending it relative to the sumulator space
        self.set_home_position(lon0,lat0, 0)

        # Done: retrieve current global position
        global_pos = self.global_position
        #lat = self._latitude
        #lon = self._longitude
        #alt = self._altitude
        
        # Done: convert to current local position using global_to_local()
        # Upon first run, this should be very close to 0,0 because the simulator starts with the drone in the exact place as the colliders.csv
        # After first run, the drone could be anywhere, so we need to set the grid home to the current location so we can move from there to some place else
        local_pos = global_to_local(global_pos,self.global_home)  
               
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        #print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
           
        # Define starting point on the grid (this is just grid center)
        # grid_start = (-north_offset, -east_offset)
        # Done: convert start position to current position rather than map center     
        grid_start = (int(local_pos[0])-north_offset, int(local_pos[1])-east_offset)

        # Set goal as some arbitrary position on the grid
        # grid_goal = (-north_offset + 10, -east_offset + 10)  
        # Done: adapt to set goal as latitude / longitude position and convert
        # global_goal - np.array([-122.395606, 37.793719, 5])
        # global_goal = np.array([-122.395216, 37.795063, 5]) #Justin Herman Plaza
        # global_goal = np.array([-122.397619, 37.793444, 5]) #One block
        global_goal = np.array([-122.396064, 37.793991, 5]) #Corner of Drumm and California
        # global_goal = np.array([-122.396270, 37.793590, 5]) #Down a street
        global_goal = np.array([-122.397222, 37.795100, 5]) # A few blocks over
        # global_goal = np.array([-122.397550, 37.792550, 5]) # Point A
        # global_goal = np.array([-122.397450, 37.792480, 5]) # Point B
        
        local_goal = global_to_local(global_goal,self.global_home)
        grid_goal = (int(local_goal[0])-north_offset , int(local_goal[1])-east_offset)
        print("local goal" , local_goal)
                    
        # Run A* to find a path from start to goal
        # Done: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        #print('Local Start and Goal: ', grid_start, grid_goal)
        
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        # Done: prune path to minimize number of waypoints
        # print(path)
        if len(path) > 8: # Delete colinear waypoints
            p1 = path[0]
            p2 = path[1]
            for point in path[2:-1]:
                p3 = point
                if self.collinearity_int(p1, p2, p3, 1):
                    path.remove(p2)
                else:
                    p1 = p2
                p2 = p3

        # Nope (if you're feeling ambitious): Try a different approach altogether!
   
        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        #waypoints = [[p[0] , p[1] , TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        
        # Done already: send waypoints to sim (this is just for visualization of waypoints)
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
