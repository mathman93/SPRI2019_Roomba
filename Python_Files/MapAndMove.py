''' MapAndMove.py
Purpose: Draws a virtual map with the origin and the coordinates given, and moves from the start to its goal. Also adds points that it bumps into to the map as walls
     that it will attempt to move around to get to the goal, and keep the walls in memory for its movement in the future
Last Modified: 7/10/2019
'''

## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO
import RoombaCI_lib
import os.path
import math
import heapq

## Variables and Constants ##
# LED pin numbers
yled = 5
rled = 6
gled = 13

## Functions and Definitions ##
''' Displays current date and time to the screen
    '''
def DisplayDateTime():
    # Month day, Year, Hour:Minute:Seconds
    date_time = time.strftime("%B %d, %Y, %H:%M:%S", time.gmtime())
    print("Program run: ", date_time)

''' Queue that allows for elements with a higher priority (lower priority value) to be put closer to the front of the line
    '''
class PriorityQueue:
    def __init__(self):
        self.elements = []
    # Returns True if queue is empty, otherwise it returns false
    def empty(self):
        return len(self.elements) == 0
    # Put x into the queue
    def put(self, item, priority):
        heapq.heappush(self.elements,(priority,item))
    # Takes first thing from list
    def get(self):
        return heapq.heappop(self.elements)[1]

''' Defines a world, finds the neighbors of certain points, and the location of thos points
    '''
class GridWorld:
    def __init__(self):
        # Points that can connect to other points in the world
        self.edges = {}
        # Points that exist in the world
        self.points = []
        # Walls that are found in the world
        self.walls = []
    # Tells you which points are able to be connected to
    # Note: ID needs to be a tuple 
    def neighbors(self,id):
        return self.edges.get(id,[])
    # Gives the physical world location 
    def Location(self,id):
        x_pos = (id[0]*300)+150
        y_pos = (id[1]*300)+150
        return(x_pos,y_pos)
    def removePointFromWorld(self,xy): # Removes the point from the world at the specified tuple 'xy' from the world 'MyWorld'
        neighborlist = self.edges.pop(xy)
        for p in neighborlist:
            self.edges[p].remove(xy)
        self.points.remove(xy)
    def addEdgeToWorld(self,point1,point2): # Adds an edge from the world between two given tuple coordinate points
        print("{0},{1}".format(point1,point2))
        if point1 in self.points and point2 in self.points:
            ls1 = self.edges[point1]
            ls1.append(point2)
            self.edges[point1] = ls1
            ls2 = self.edges[point2]
            ls2.append(point1)
            self.edges[point2] = ls2
        else:
            print("Point is not in world")
    def removeEdgeFromWorld(self,point1,point2): # Removes an edge from the world between two given tuple coordinate points
        if point1 in self.points and point2 in self.points:
            p1 = self.edges[point1]
            p1.remove(point2)
            self.edges[point1] = p1
            p2 = self.edges[point2]
            p2.remove(point1)
            self.edges[point2] = p2
        else:
            print("Point is not in world")

''' Calculates euclidian distance between two given tuple coordinate points
    '''
def distance(p1,p2):
    return math.sqrt((p2[0]-p1[0])**2+(p2[1]-p1[1])**2)

''' Creates a grid world with the two given "start" and "goal" locations
    '''
def makeworld(start,goal):
    # Call the class gridworld
    MyWorld = GridWorld()
    MyWorld.points.append(start)
    MyWorld.edges[start] = []
    MyWorld.points.append(goal)
    MyWorld.edges[goal] = []
    MyWorld.addEdgeToWorld(start,goal)
    return MyWorld
'''
    # Create points that exist in the world
    for x in range(x_range):
        for y in range(y_range):
            MyWorld.points.append((x,y))
    # Finds the neighbors of the points and determines if they are in the world
    for point in MyWorld.points:
        x = point[0]
        y = point[1]
        # All eight directions around the position of the roomba. Could do just the four next to the point.
        neighbors = [(x+1,y+1),(x+1,y),(x+1,y-1),(x,y-1),(x,y+1),(x-1,y-1),(x-1,y),(x-1,y+1)]
        group = []
        # Checking points to make sure that they are in the world 
        for point1 in neighbors:
            if point1 in MyWorld.points:
                group.append(point1)
        # Update points that are connect by edges
        MyWorld.edges[point] = group
    return MyWorld
    '''

''' Uses A* method of pathfinding to find best path the fastest between a given start and goal on the given plane "MyWorld"
    '''
def A_star(start,goal,MyWorld):
    # frontier of points that havent been searched
    frontier = PriorityQueue()
    # put a point in frontier
    frontier.put(start,0)
    # dictionary of squares that other squares came from
    came_from = {}
    came_from[start]=None
    cost_so_far={}
    cost_so_far[start]=0
    while not frontier.empty():
        current = frontier.get()
        print("Searching {0}".format(current))
        # If the place we are at is the goal end the search
        if current == goal:
            break
        # Search for each point that is next to current
        for next in MyWorld.neighbors(current):
            new_cost = cost_so_far[current]+distance(current,next) + angle_cost(came_from[current],current,next)
            if new_cost < cost_so_far.get(next,math.inf):
                cost_so_far[next]=new_cost
                priority = new_cost+distance(next,goal)
                frontier.put(next,priority)
                came_from[next] = current

    # Find the path from the start to end
    current=goal
    path = []
    while  current!=start:
        print("Current: {0}".format(current))
        path.append(current)
        current=came_from[current]

    path.append(start)
    path.reverse()
    return path

''' Calculates a cost used to determine the past path in regards to how the Roomba rotates according to its previous, current, and next locations in its path
    '''
def angle_cost(previous,current,next):
    if previous == None: # If first movement...
        return 0
    else:
        theta = math.atan2(current[1]-previous[1],current[0]-previous[0])
        theta_initial = math.atan2(next[1]-current[1],next[0]-current[0])
        theta_d = theta_initial - theta
        if theta_d > math.pi:
            theta_d -= 2*math.pi
        elif theta_d <= -math.pi:
            theta_d += 2*math.pi
        return abs(theta_d)

''' Finds if it is possible for a path to be formed without intersecting a circle drawn around a given wall point
    '''
def CanMakeEdge(start,goal,wall):
    x1 = start[0]
    x2 = goal[0]
    y1 = start[1]
    y2 = goal[1]
    xc = wall[0]
    yc = wall[1]


    if y1 == y2:
        x=xc
        y=y1
        outside = (x<x1 and x<x2) or (x>x1 and x>x2)
    elif x1==x2:
        x = x1
        y = yc
        outside = (y<y1 and y<y2) or (y>y1 and y>y2)
    else:
        m=(x2-x1)/(y2-y1)
        b1 = x1 - m*y1
        b2 = xc + (yc/m)
        y = (b2-b1)/(m+(1/m))
        x = m*y+b1
        outside = (x<x1 and x<x2) or (x>x1 and x>x2)
    print(x)
    print(y)
    if outside:
        return True
    elif distance((x,y),(xc,yc))>200:
        return True
    else:
        return False

''' Function that returns the angle of an object (in degrees in the range 0-360) that the roomba is bumping into.
    Uses the 'bumper' bumper reading (query code 7) and the 'l_bumper' light bumper reading (query code 45), returns angles in range of -70,-45,-20,0,20,45,70
    '''
def BumpAngle(bumper,l_bumper):
    l_bumper_list = []
    for i in range(6): #For all the possible binary digits that represent light bumpers being activated...
        if l_bumper & pow(2,i) == pow(2,i): #If the light bumper value indicates that the "i" light bumper is being triggered...
            l_bumper_list.append(True) # The placeholder boolean for that light bumper is set to true
        else: # If the "i" light bumper is not being triggered...
            l_bumper_list.append(False) # The placeholder boolean is set to false
    print(l_bumper_list)
    [L,FL,CL,CR,FR,R] = l_bumper_list # Sets the booleans to be used in conditionals to their own variables for easy reference
    if bumper == 3: # If roomba detects a bump in the center...
        return 0
    if bumper == 1: # If the roomba detects a bump on the right...
        if not (CL or CR or FR): # If the center two or front right light bumpers are not triggered at all...
            return math.radians(70)
        elif CL: # If the center left light bumper is triggered...
            return math.radians(20)
        else:
            return math.radians(45)
    if bumper == 2: # If roomba detects a bump on the left...
        if not (L or FL or CL or CR): # If any of the left side or center right light bumpers are not triggered...
            return math.radians(-70)
        elif CR: # If the center right light bumper is triggered...
            return math.radians(-20)
        else: 
            return math.radians(-45)
## -- Code Starts Here -- ##
# Setup Code #
GPIO.setmode(GPIO.BCM) # Use BCM pin numbering for GPIO
DisplayDateTime() # Display current date and time

# LED Pin setup
GPIO.setup(yled, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(rled, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(gled, GPIO.OUT, initial=GPIO.LOW)

# Wake Up Roomba Sequence
GPIO.output(gled, GPIO.HIGH) # Turn on green LED to say we are alive
print(" Starting ROOMBA... ")
Roomba = RoombaCI_lib.Create_2("/dev/ttyS0", 115200)
Roomba.ddPin = 23 # Set Roomba dd pin number
GPIO.setup(Roomba.ddPin, GPIO.OUT, initial=GPIO.LOW)
Roomba.WakeUp(131) # Start up Roomba in Safe Mode
# 131 = Safe Mode; 132 = Full Mode (Be ready to catch it!)
Roomba.BlinkCleanLight() # Blink the Clean light on Roomba

if Roomba.Available() > 0: # If anything is in the Roomba receive buffer
    x = Roomba.DirectRead(Roomba.Available()) # Clear out Roomba boot-up info
    #print(x) # Include for debugging

print(" ROOMBA Setup Complete")
GPIO.output(gled, GPIO.LOW)

start_time = time.time()
[left_start,right_start]=Roomba.Query(43,44)

# Variables and Constants
backup_time = 1.0 # Amount of time spent backing up
corner_time = 1.5 # Amount of time that it takes before the roomba starts turning more sharply (makes sure it turns around corners)
f = 0 # Forward/Backward speed
s = 0 # Rotation Speed
bump_time = time.time() - 3.0 # Assures that the roomba doesn't start in backup mode
bump_count = 0 # Keeps track of how many times the roomba has bumped into a wall
theta = 0 # Current heading
wheel_diameter = 72
counts_per_rev = 508.8
distance_between_wheels = 235
C_theta = (wheel_diameter*math.pi)/(counts_per_rev*distance_between_wheels)
distance_per_count = (wheel_diameter*math.pi)/counts_per_rev
data_time = time.time()
bump_mode = False # Used to tell whether or not the roomba has bumped into something and is supposed to be "tracking"
bump_code = 0 # Used to distinguish if the right, left, or center bumpers are being triggered
bump_count = 0 # Keeps track of how many times the bumper has detected a bump
new_points = [0,0,0] # Initializes list of new points to be used to recalculated path after bumping into object

while True: #Loop that asks for initial x and y coordinates
    try:
        x_final = int(input("X axis coordinate:"))
        y_final = int(input("Y axis coordinate:"))
        break
    except ValueError:
        print("Please input a number")
        continue

y_position = 0 # Current position on the y-axis
x_position = 0 # Current position on the x-axis
start = (0,0) # Starting position in the MyWorld grid
goal = (x_final,y_final) # Final goal
MyWorld = makeworld(start,goal) # Creates a grid world for the roomba to move in with two points, the start and goal, and draws a line between them
path = A_star(start,goal,MyWorld) # Creates the optimal pathway between the start and goal
current_point = start # Saves grid coordinate that the roomba just came from
bump_break = False # Checks if the roomba has bumped into something and broken out of the loop

#Print Stuff
print(path)
#for point in MyWorld.edges.keys():
#   value = MyWorld.edges[point]
#   print("{0}:{1}".format(point,value))

#print(MyWorld.neighbors((10,1)))
#print(MyWorld.neighbors((5,1)))
#print(MyWorld.Location((5,2)))

while True:
    for point in path:
        current_goal = point
        distance_to_end = math.sqrt((current_goal[0]-x_position)**2 +(current_goal[1]-y_position)**2) # Distance of straight line between where the roomba is and where the end point is
        theta_initial = math.atan2((current_goal[1]-y_position),(current_goal[0]-x_position)) # Angle of the line between the x-axis and the initial distance to end line
        theta_d = theta_initial-theta # Rotation needed from current heading to face goal
        #print("{0:.6f},{1},{2},{3:.3f},{4:.3f},{5:.6f},{6:.6f},{7:.6f}".format(time.time()-data_time,left_start,right_start,x_position,y_position,theta,distance_to_end,theta_d))
        Roomba.StartQueryStream(7,43,44,45) # Start getting bumper values
        try:
            while distance_to_end > 3:
                if Roomba.Available()>0:
                    data_time2 = time.time()
                    # Get bump value, then get left and right encoder values and find the change in each
                    [bump,left_encoder, right_encoder, l_bump] = Roomba.ReadQueryStream(7,43,44,45)
                    delta_l = left_encoder-left_start
                    if delta_l < -1*(2**15): #Checks if the encoder values have rolled over, and if so, subtracts/adds accordingly to assure normal delta values
                        delta_l += (2**16)
                    elif delta_l > (2**15):
                        delta_l -+ (2**16)
                    delta_r = right_encoder-right_start
                    if delta_r < -1*(2**15):
                        delta_r += (2**16)
                    elif delta_r > (2**15):
                        delta_r -+ (2**16)
                    # Determine the change in theta and what that is currently
                    delta_theta = (delta_l-delta_r)*C_theta
                    old_theta = theta # Heading of last iteration
                    theta += delta_theta
                    # If theta great than 2pi subtract 2pi and vice versus. Normalize theta to 0-2pi to show what my heading is.
                    if theta >= 2*math.pi:
                        theta -= 2*math.pi
                    elif theta < 0:
                        theta += 2*math.pi
                    # Determine what method to use to find the change in distance
                    if delta_l-delta_r == 0:
                        delta_d = 0.5*(delta_l+delta_r)*distance_per_count
                    else:
                        delta_d = 2*(235*(delta_l/(delta_l-delta_r)-.5))*math.sin(delta_theta/2)                        

                    # Find new x and y position, and their integer versions
                    x_position = x_position + delta_d*math.cos(theta-.5*delta_theta)
                    y_position = y_position + delta_d*math.sin(theta-.5*delta_theta)
                    x_pos_int = int(x_position)
                    y_pos_int = int(y_position)
                    # Find distance to end and theta_initial
                    distance_to_end = math.sqrt((current_goal[0]-x_position)**2 +(current_goal[1]-y_position)**2)

                    theta_initial = math.atan2((current_goal[1]-y_position),(current_goal[0]-x_position))
                    # Normalize what theta initial is to between 0-2pi
                    if theta_initial <0:
                        theta_initial += 2*math.pi
                    # Calculate theta_d and normalize it to 0-2pi
                    # This value is the difference between the direction were supposed to be going and the direction we are going
                    theta_d = ((theta_initial-theta)%(2*math.pi))
                    # get theta_d between -pi and pi
                    if theta_d > math.pi:
                        theta_d -= 2*math.pi

                    # Checks if the roomba is bumping into something, and if so, activates wall detection protocol
                    if(bump%4) > 0: # If the roomba bumps into something...
                        bump_count += 1 # Updates the amount of times the bumpers have detected a bump
                        wall_dir = BumpAngle(bump,l_bump) # Keeps track of what the light bumper detected when the roomba bumped
                        print('Bump Angle: {0:.4f}'.format(wall_dir))
                        if bump_count == 1:# If first bump in the cycle...
                            MyWorld.removeEdgeFromWorld(current_point,current_goal) # Remove the edge from the previous starting point to the goal
                            x_wall = int(x_pos_int + (200*math.cos(theta + wall_dir))) # Calculates x position of wall
                            y_wall = int(y_pos_int + (200*math.sin(theta + wall_dir))) # Calculates y position of wall
                            MyWorld.walls.append((x_wall,y_wall)) # Adds the coordinate position of the wall to the list of walls
                            points_to_remove = []
                            print("New Wall Made: {0}".format((x_wall,y_wall)))
                            # Removes all points too close to new wall
                            for point in MyWorld.points:
                                if distance(point,(x_wall,y_wall)) < 200:
                                    points_to_remove.append(point)
                            for p in points_to_remove:
                                MyWorld.removePointFromWorld(p)
                        bump_time = time.time() #Sets up timer that tells how long to back up

                    if time.time() - bump_time < 2.0: # If has bumped into something less than 2 seconds ago, back up
                        f = -100
                        s = 0
                    elif time.time() - bump_time < 2.5: # If done backing up...
                        current_point = (x_pos_int,y_pos_int)
                        bump_break = True # Validates that the roomba has broken out of the loop
                        new_points[0] = (x_pos_int,y_pos_int) # Current point after backing up from wall
                        np1x = int(x_pos_int + (400 * math.cos(theta+wall_dir+(math.pi/2)))) # X position of point to right of roomba
                        np1y = int(y_pos_int + (400 * math.sin(theta+wall_dir+(math.pi/2)))) # Y position of point to right of roomba
                        new_points[1] = (np1x,np1y)
                        np2x = int(x_pos_int + (400 * math.cos(theta+wall_dir-(math.pi/2)))) # X position of point to left of roomba
                        np2y = int(y_pos_int + (400 * math.sin(theta+wall_dir-(math.pi/2)))) # Y position of point to left of roomba
                        new_points[2] = (np2x,np2y)
                        break

                    else: # If haven't bumped into anything yet...
                        if abs(theta_d) > (math.pi / 4): #If theta_d is greater than pi/4 radians...
                            s_set = 100 # Spin faster
                        elif abs(theta_d) > (math.pi / 36): #If theta_d is getting closer...
                            s_set = 60 # Spin normal speed
                        else: # otherwise, if theta_d is fairly small
                            s_set = 20 # Spin slow
                        if distance_to_end > 150: #If distance_to_end is greater than 150 mm...
                            f_set = 120 #Go faster
                        elif distance_to_end > 50: # If distance_to_end is greater than 50 mm...
                            f_set = 80 #Go fast
                        else: #otherwise, if distance_to_end is less than 50 mm...
                            f_set = 40 #Go slow     

                        radius = ((235 / 2) * (f_set / s_set)) #Radius of circle of the roomba's turn for the given f_set and s_set values

                        if theta_d > 0: #Rotates clockwise if theta_d is positive
                            s = s_set
                        elif theta_d < 0: #Rotates counterclockwise if theta_d is negative
                            s = s_set * -1
                        else:
                            s = 0
                        if theta_d > (math.pi / 3) or theta_d < (math.pi / -3): #If the end point is beyond 90 degrees in either direction, the roomba will rotate in place
                            f = 0
                        elif abs(2*radius*math.sin(theta_d)) > distance_to_end: #If the end point is within the circle that is drawn by the roomba's turn path, then the roomba will rotate in place 
                            f = 0
                        else:
                            f = f_set
                    Roomba.Move(f,s) # Move with given forward and spin values
                    #print("{0:.6f},{1},{2},{3:.3f},{4:.3f},{5:.6f},{6:.6f},{7:.6f}, bump_count:{8}".format(data_time2-data_time,left_start,right_start,x_position,y_position,theta,distance_to_end,theta_d,bump_count))
                    left_start = left_encoder
                    right_start = right_encoder
            Roomba.PauseQueryStream() #Pauses the query stream while new coordinates are being input
            if Roomba.Available()>0:
                z = Roomba.DirectRead(Roomba.Available())
                print(z)
            Roomba.Move(0,0)
            if bump_break: # If had to break out of the loop after bumping...
                break
            current_point = point
        except KeyboardInterrupt:
            break
    if bump_break: # If the roomba has bumped into something and broken out of the loop...
        # Reset variables responsible for bumping operations
        bump_break = False
        bump_time = time.time() - 3.0
        bump_count = 0
        new_list = [] # List of points viable for the roomba to move to after bumping into an object
        for p1 in new_points: # Check if the current point, point to the left, or point to the right are not too close to another point or too close to a current wall
            point_check = True
            '''
            for p2 in MyWorld.points:
                if distance(p1,p2) < 10:
                    point_check = False
                    break
            '''
            for wall in MyWorld.walls:
                if distance(p1,wall) < 200:
                    point_check = False
                    break
            if point_check == True:
                new_list.append(p1)
        for point in new_list:
            MyWorld.points.append(point)
            MyWorld.edges[point] = []
        print("Points: {0}".format(MyWorld.points))
        print("new_list: {0}".format(new_list))
        for p1 in new_list: # Check if any of the cleared points from the last loop can be moved to
            for p2 in MyWorld.points:
                point_check = True
                if p2 != p1:
                    for wall in MyWorld.walls:
                        if CanMakeEdge(p1,p2,wall) == False:
                            point_check = False
                            break
                    if point_check == True:
                        MyWorld.addEdgeToWorld(p1,p2)
                        print("Made an edge")
        print("Points: {0}".format(MyWorld.points))
        print("new_list: {0}".format(new_list))
        for point in MyWorld.edges.keys():
            value = MyWorld.edges[point]
            print("{0}:{1}".format(point,value))
        print("World Walls: {0}".format(MyWorld.walls))
        path = A_star(current_point,goal,MyWorld) # Generate a new path with updated walls, points, and edges
    else:
        print("World Points: {0}".format(MyWorld.points))
        for point in MyWorld.edges.keys():
            value = MyWorld.edges[point]
            print("{0}:{1}".format(point,value))
        print("World Walls: {0}".format(MyWorld.walls))
        start = goal
        while True: #Loop that asks for initial x and y coordinates
            try:
                x_final = int(input("X axis coordinate:"))
                y_final = int(input("Y axis coordinate:"))
                goal = (x_final,y_final)
                if goal not in MyWorld.points:
                    goal_check = True
                    for wall in MyWorld.walls:
                        if distance(goal,wall) < 200:
                            print("Too close to a wall")
                            goal_check = False
                            break
                    if goal_check:
                        MyWorld.edges[goal] = []
                        MyWorld.points.append(goal)
                        for p in MyWorld.points:
                            goal_check = True
                            if p != goal:
                                for wall in MyWorld.walls:
                                    if CanMakeEdge(p,goal,wall) == False:
                                        goal_check = False
                                        break
                                if goal_check:
                                    MyWorld.addEdgeToWorld(p,goal)
                                    break
                    else:
                        continue
                break
            except ValueError:
                print("Please input a number")
                continue
        print("Points: {0}".format(MyWorld.points))
        print("new_list: {0}".format(new_list))
        for point in MyWorld.edges.keys():
            value = MyWorld.edges[point]
            print("{0}:{1}".format(point,value))
        print("World Walls: {0}".format(MyWorld.walls))
        path = A_star(start,goal,MyWorld)
        print(path)

Roomba.Move(0,0)
Roomba.PauseQueryStream()
if Roomba.Available()>0:
    z = Roomba.DirectRead(Roomba.Available())
    print(z)
time.sleep(0.1)
## -- Ending Code Starts Here -- ##
Roomba.ShutDown() # Shutdown Roomba serial connection
GPIO.cleanup() # Reset GPIO pins for next program