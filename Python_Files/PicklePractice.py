''' PicklePractice.py
Purpose: Practice with pickling (not to be confused with pickleball practice)
Last Modified: 7/17/2019
'''

## Import libraries ##
import pickle
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

''' Queue that allows for elements with a higher priority (lower priority value) to be put closer to the front of the line.
    Created using documentation from https://www.redblobgames.com/pathfinding/a-star/implementation.html
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

''' Defines a world, finds the neighbors of certain points, and the location of thos points.
    Created using documentation from https://www.redblobgames.com/pathfinding/a-star/implementation.html
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
    # Removes the point from the world at the specified tuple 'xy' from the world 'MyWorld'
    def removePointFromWorld(self,xy):
        neighborlist = self.edges.pop(xy)
        for p in neighborlist:
            self.edges[p].remove(xy)
        self.points.remove(xy)
    # Adds an edge from the world between two given tuple coordinate points
    def addEdgeToWorld(self,point1,point2):
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
    # Removes an edge from the world between two given tuple coordinate points
    def removeEdgeFromWorld(self,point1,point2):
        if point1 in self.points and point2 in self.points:
            p1 = self.edges[point1]
            p1.remove(point2)
            self.edges[point1] = p1
            p2 = self.edges[point2]
            p2.remove(point1)
            self.edges[point2] = p2
        else:
            print("Point is not in world")
    # Inserts a point into the world and attaches edges to it from all other points as long as there is not a wall in the way
    def integrateIntoWorld(self, point):
        self.edges[point] = [] # Create an edge dictionary entry for the point
        self.points.append(point) # Add point to the world
        for p in self.points:
            point_check = True
            if p != point: # If the points aren't the same...
                for wall in self.walls:
                    if CanMakeEdge(p,point,wall) == False: # If cannot make edge between the two points...
                        point_check = False # Don't make an edge
                        break
                if point_check: # If can make an edge...
                    self.addEdgeToWorld(p,point) # Add the edge between the points

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
        theta = math.atan2(current[1]-previous[1],current[0]-previous[0]) # Finds current rotation
        theta_initial = math.atan2(next[1]-current[1],next[0]-current[0]) # Finds angle of rotation towards next goal
        theta_d = theta_initial - theta # Finds difference between the two angles
        # Normalizes difference between 0 to 2pi
        if theta_d > math.pi:
            theta_d -= 2*math.pi
        elif theta_d <= -math.pi:
            theta_d += 2*math.pi
        return abs(theta_d)

''' Finds if it is possible for a path to be formed without intersecting a circle drawn around a given wall tuple coordinate
    '''
def CanMakeEdge(start,goal,wall):
    #Gets values for vectors from start to goal and from start to wall
    ax = goal[0] - start[0]
    ay = goal[1] - start[1]
    bx = wall[0] - start[0]
    by = wall[1] - start[1]
    # Normalizes both vectors
    norm_a = math.sqrt((ax**2)+(ay**2))
    dot_ab = (ax*bx) + (ay*by)
    b1 = dot_ab / norm_a
    norm_b = math.sqrt((bx**2)+(by**2))
    # Checks if line can be drawn from start to goal and won't collide with wall
    if b1 >= norm_a or b1 <= 0:
        return True
    elif (norm_b**2) - (b1**2) > (200**2):
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

''' Returns a list of points generated in a spiral from the given starting coordinate (x_pos_init,y_pos_init) using the 'unit' length between points
    up to the limit of times of 'threshold'
    '''
def SpiralPath(x_pos_init,y_pos_init,threshold,unit):
    count = 0
    path = [(x_pos_init,y_pos_init)]
    while count < threshold:
        count += 1
        for n in range(count):
            next_x = (path[-1])[0] + pow(-1,count+1)*unit
            path.append((next_x,(path[-1])[1]))
        for n in range(count):
            next_y = (path[-1])[1] + pow(-1,count+1)*unit
            path.append(((path[-1])[0],next_y))
    return path

''' Returns the next coordinate point in a spiral pattern using the last point 'previous_point' that it was at
    '''
def NextCoordinate(previous_point,unit):
    (x,y) = previous_point
    N = max(abs(x),abs(y)) # Find maximum value from x and y coordinates to find the side of the 'square' made by the spiral
    if (x == N and y == -N) or (y == -N and x < N):
        return (x+unit,y) # Move up to next radius of spiral / Move upwards
    if x == N and y < N:
        return (x,y+unit) # Move right
    if y == N and x > -N:
        return (x-unit,y) # Move down
    if x == -N and y > -N:
        return (x,y-unit) # Move left

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

points = [(0,0),(100,0),(100,100),(0,100)]
edges = {(0,0):(100,0),(100,100),(0,100)}
walls = [(-100,100)]

with open(file_name, "wb") as file:
    pickle.dump(points,file)
    pickle.dump(edges,file)
    pickle.dump(walls,file)
if Roomba.Available()>0:
    z = Roomba.DirectRead(Roomba.Available())
    print(z)
time.sleep(0.1)
#file.close()
## -- Ending Code Starts Here -- ##
Roomba.ShutDown() # Shutdown Roomba serial connection
GPIO.cleanup() # Reset GPIO pins for next program