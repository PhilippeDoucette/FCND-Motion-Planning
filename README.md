## Project: 3D Motion Planning

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
***planning_Utils*** includes: 

 a_star: the path plannng routine that takes a grid, heuristic, and start/end points and returns a path through the obstacles defined in the grid.
  
 heuristic: A frobenius norm for the distance between the current point and the goal.
  
 Enum Action: comprised of valid moves within the grid.  I expanded the four directions to include compass diagonals.
  
 valid_actions: Returns a list of valid actions for the current node.  I expanded this to include compass diagonals.
  
 create_grid: Returns the grid used by a_star for a given altitude and safety distance.
 
***motion_planning*** includes:
 Three callbacks for local position, velocity and state, which provide the events wich trigger the transition routines
 Six transitions, arming, takeoff, waypoint, landing disarming, and manual, wich advance the vehicle state and run the appropriate methods in the underlying Drone class, such as arm(), takeoff(), etc.
 plan_path, which is called right after arming, calculates the set of waypoints, by using the a_star path planning
 
 

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
I used a python open statement to read the first text line of colliders.csv.  I brute-force parced the text into variables. In production, I would not rely on strict text location if this file were to be edited by people.  

        f = open('colliders.csv')
        line = f.readline()
        lat0, lon0 = float(line[5:14]), float(line[21:])
        print("lat {0}   lon {1}".format(lat0, lon0))

#### 2. Set your current local position
With the values from the prior step, I set the global home position.

        self.set_home_position(lon0,lat0, 0)

        
#### 3. Set grid start position from local position
With the global home position set, I used it to set the local position so that I can always use the current position as the starting point for multiple runs of the simulator.  All I have to do is set a new goal and it starts from it's current location.

        global_pos = self.global_position
        local_pos = global_to_local(global_pos,self.global_home) 
                ...
        grid_start = (int(local_pos[0])-north_offset, int(local_pos[1])-east_offset)

#### 4. Set grid goal position from geodetic coords
It is easiler to supply lat-lon positions from the sumulator as goal positions.  We can then fly around manually to an interesting spot, record the lat-lon, and fly back there later. I created a list I can un-comment to fly to a location.  This also works with Google Maps by specifying a lat-lon

        # global_goal - np.array([-122.395606, 37.793719, 5])
        # global_goal = np.array([-122.395216, 37.795063, 5]) # Justin Herman Plaza
        # global_goal = np.array([-122.397619, 37.793444, 5]) # One block
        global_goal = np.array([-122.396064, 37.793991, 5]) # Corner of Drumm and California
        # global_goal = np.array([-122.396270, 37.793590, 5]) # Down a street
        # global_goal = np.array([-122.397222, 37.795100, 5]) # A few blocks over
        # global_goal = np.array([-122.397550, 37.792550, 5]) # Point A
        # global_goal = np.array([-122.397450, 37.792480, 5]) # Point B

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
I modified the enum class Action to include diagonal moves and their costs
  
    class Action(Enum):
      WEST = (0, -1, 1)
      EAST = (0, 1, 1)
      NORTH = (-1, 0, 1)
      SOUTH = (1, 0, 1)
      NORTHEAST = (-1, 1, 1.41)
      SOUTHEAST = (1, 1, 1.41)
      NORTHWEST = (-1, -1, 1.41)
      SOUTHWEST = (1, -1, 1.41)
    
      ... 

It was also necessary to modify valid_actions to check grid bondaries and obstacle collisions for diagonal moves

def valid_actions(grid, current_node):

    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)

    if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTHEAST)
    if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTHEAST)
    if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTHWEST)
    if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTHWEST)
    return valid_actions

#### 6. Cull waypoints 
To eliminate the redundant waypoints, I loop through the path and check for collinearity against a threshold value 

     path, _ = a_star(grid, heuristic, grid_start, grid_goal)
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

Note that it relies on a boolean function call with epsilon as a threshold, which I set to '1' but could be tuned. If I were to advance this code, I would make the test floating point numbers.

    def collinearity_int(self,p1, p2, p3, epsilon): 
        collinear = False
        det = p1[0]*(p2[1] - p3[1]) + p2[0]*(p3[1] - p1[1]) + p3[0]*(p1[1] - p2[1])
        print(det)
        if abs(det) <= epsilon:
            collinear = True
        return collinear

### Execute the flight
#### 1. Does it work?
It works wonderfully!

![Quad Image](./FCND%20Simulator%20Drone%20Flying.jpg)


