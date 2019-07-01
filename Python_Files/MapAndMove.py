''' Mapping.py
Purpose: Display a virtual map
Last Modified: 7/1/2019
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


# defines a world, finds the neighbors of certain points, and the location of thos points
class GridWorld:
	def __init__(self):
		# Points that can connect to other points in the world
		self.edges = {}
		# Points that exist in the world
		self.points = []
	# Tells you which points are able to be connected to
	# Note: ID needs to be a tuple 
	def neighbors(self,id):
		return self.edges.get(id,[])
	# Gives the physical world location 
	def Location(self,id):
		x_pos = (id[0]*300)+150
		y_pos = (id[1]*300)+150
		return(x_pos,y_pos)


def distance(p1,p2):
	return math.sqrt((p2[0]-p1[0])**2+(p2[1]-p1[1])**2)

def makeworld(x_range,y_range):
	# Call the class gridworld
	MyWorld = GridWorld()
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

# Uses A* method of pathfinding to find best path the fastest
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
		path.append(current)
		current=came_from[current]

	path.reverse()
	return path

def angle_cost(previous,current,next): # Calculates a cost used to determine the past path in regards to how the Roomba rotates
	if previous == None: # If first movement...
		return 0
	else:
		theta = math.atan2(current[1]-previous[1],current[0]-previous[0])
		if theta < 0:
			theta += 2*math.pi
		elif theta >= 2*math.pi:
			theta -= 2*math.pi
		theta_initial = math.atan2(next[1]-current[1],next[0]-current[0])
		if theta_initial < 0:
			theta_initial += 2*math.pi
		elif theta_initial >= 2*math.pi:
			theta_initial -= 2*math.pi
		theta_d = (abs(theta_initial - theta))%(2*math.pi)
		if theta_d > math.pi:
			theta_d -= 2*math.pi
		return theta_d

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
bump_time = time.time() - 2.0 # Assures that the roomba doesn't start in backup mode
bump_count = 0 # Keeps track of how many times the roomba has bumped into a wall
y_position = 100 # Current position on the y-axis
x_position = 100 # Current position on the x-axis
theta = 0 # Current heading
wheel_diameter = 72
counts_per_rev = 508.8
distance_between_wheels = 235
C_theta = (wheel_diameter*math.pi)/(counts_per_rev*distance_between_wheels)
distance_per_count = (wheel_diameter*math.pi)/counts_per_rev
data_time = time.time()
bump_mode = False # Used to tell whether or not the roomba has bumped into something and is supposed to be "tracking"
bump_code = 0 # Used to distinguish if the right, left, or center bumpers are being triggered

while True: #Loop that asks for initial x and y coordinates
	try:
		x_final = float(input("X axis coordinate:"))
		y_final = float(input("Y axis coordinate:"))
		break
	except ValueError:
		print("Please input a number")
		continue

start = (0,0)
goal = (x_final,y_final)
MyWorld = makeworld(10,6)
path = A_star(start,goal,MyWorld)

#Print Stuff
print(path)
#for point in MyWorld.edges.keys():
#	value = MyWorld.edges[point]
#	print("{0}:{1}".format(point,value))

#print(MyWorld.neighbors((10,1)))
#print(MyWorld.neighbors((5,1)))
#print(MyWorld.Location((5,2)))

while True:
	for point in path:
		current_goal = MyWorld.Location(point)
		distance_to_end = math.sqrt((current_goal[0]-x_position)**2 +(current_goal[1]-y_position)**2) # Distance of straight line between where the roomba is and where the end point is
		theta_initial = math.atan2((current_goal[1]-y_position),(current_goal[0]-x_position)) # Angle of the line between the x-axis and the initial distance to end line
		theta_d = theta_initial-theta # Rotation needed from current heading to face goal
		print("{0:.6f},{1},{2},{3:.3f},{4:.3f},{5:.6f},{6:.6f},{7:.6f}".format(time.time()-data_time,left_start,right_start,x_position,y_position,theta,distance_to_end,theta_d))
		Roomba.StartQueryStream(7,43,44) # Start getting bumper values
		try:
			while distance_to_end > 3:
				if Roomba.Available()>0:
					data_time2 = time.time()
					# Get bump value, then get left and right encoder values and find the change in each
					[bump,left_encoder, right_encoder] = Roomba.ReadQueryStream(7,43,44)
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

					# Find new x and y position
					x_position = x_position + delta_d*math.cos(theta-.5*delta_theta)
					y_position = y_position + delta_d*math.sin(theta-.5*delta_theta)
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

					# Checks if the roomba is currently bumping into something; if so, sets up bump mode or corner mode
					if(bump%4) > 0: # If the roomba bumps into something...
						bump_time = time.time() #Sets up timer
						bump_mode = True # Keeps in memory that the roomba will now try and track the object it bumped into
						bump_count += 1 # Increases whenever the roomba bumps into something, increases by about ten whenever a bump occurs
						if bump_count < 15: # If it is the first bump...
							bump_code = (bump%4) # Remembers if left/right/center bump for the current object/wall
						theta_threshold = theta # Remembers what heading was when bumped
					# Once the roomba has gone in a certain direction for long enough, sets the roomba to try the otehr direction
					if bump_count > 100: # When the roomba has bumped in one direction for more than about ten times...
						if time.time() - bump_time < 5.0: # For five seconds...
							f = -25 # Go backwards
							if bump_code == 1: # If bump right...
								s = 75 # Rotate clockwise
							if bump_code == 2 or bump_code == 3: # If bump left or center...
								s= -75 # Rotate counterclockwise
						else: # After five seconds is up...
							bump_count = 0 # Reset bump count
					# Tells the roomba to move backwards and rotate sharply on its first bump
					elif bump_mode and time.time() - bump_time < backup_time and bump_count < 15: # If hasn't backed up for long enough and it's the first bump...
						f = -50 #Back up
						if bump_code == 1: # If bump right...
							s = -150 #Spin counterclockwise
						if bump_code == 2 or bump_code == 3: # If bump left or center...
							s = 150 #Spin clockwise
					# Tells the roomba to move backwards and rotate less sharply on bumps after the first 
					elif bump_mode and time.time() - bump_time < backup_time: # If hasn't backed up for long enough and is not the first bump...
						f = -50 # Back up
						if bump_code == 1: # If bump right...
							s = -50 # Spin counterclockwise slower
						if bump_code == 2 or bump_code == 3: # If bump left or center...
							s = 50 # Spin clockwise slower
					# Tells the roomba to move for a short period of time after the bump in "wall mode"
					elif bump_mode and time.time() - bump_time < (backup_time + corner_time): # If not having to back up but still has bumped into something before...
						# If the roomba has turned more than 90 degrees, or the heading crossed over theta_initial...
						if theta - theta_threshold > (math.pi/2) or theta - theta_threshold < (math.pi/-2) or (theta > theta_initial and old_theta < theta_initial) or (theta < theta_initial and old_theta > theta_initial):
							bump_mode = False # Exit bump mode, and re enter "goal mode"
						else:
							f = 100 # Go forward
							if bump_code == 1: # If bump right...
								s = 15 # Turn clockwise
							if bump_code == 2 or bump_code == 3: # If bump left or center...
								s = -15 # Turn counterclockwise
					elif bump_mode and time.time() - bump_time < (backup_time + (corner_time * 1.5)): # If not having to back up and bumped into something, but been a while...
						# If the roomba has turned more than 90 degrees, or the heading crossed over theta_initial...
						if theta - theta_threshold > (math.pi/2) or theta - theta_threshold < (math.pi/-2) or (theta > theta_initial and old_theta < theta_initial) or (theta < theta_initial and old_theta > theta_initial):
							bump_mode = False
						else:
							f = 100 # Go forward
							if bump_code == 1: # If bump right...
								s = 50 # Turn more clockwise
							if bump_code == 2 or bump_code == 3: # If bump left or center...
								s = -50 # Turn more counterclockwise
					elif bump_mode:
					# If the roomba has turned more than 90 degrees, or the heading crossed over theta_initial...
						if theta - theta_threshold > (math.pi/2) or theta - theta_threshold < (math.pi/-2) or (theta > theta_initial and old_theta < theta_initial) or (theta < theta_initial and old_theta > theta_initial):
							bump_mode = False
						else:
							f = 100 # Go forward
							if bump_code == 1: # If bump right...
								s = 100 # Turn hard clockwise
							if bump_code == 2 or bump_code == 3: # If bump left or center...
								s = -100 # Turn hard counterclockwise
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
						if theta_d > (math.pi / 2) or theta_d < (math.pi / -2): #If the end point is beyond 90 degrees in either direction, the roomba will rotate in place
							f = 0
						elif abs(2*radius*math.sin(theta_d)) > distance_to_end: #If the end point is within the circle that is drawn by the roomba's turn path, then the roomba will rotate in place 
							f = 0
						else:
							f = f_set
					Roomba.Move(f,s) # Move with given forward and spin values
					print("{0:.6f},{1},{2},{3:.3f},{4:.3f},{5:.6f},{6:.6f},{7:.6f}, bump_count:{8}".format(data_time2-data_time,left_start,right_start,x_position,y_position,theta,distance_to_end,theta_d,bump_count))
					left_start = left_encoder
					right_start = right_encoder
			Roomba.PauseQueryStream() #Pauses the query stream while new coordinates are being input
			if Roomba.Available()>0:
				z = Roomba.DirectRead(Roomba.Available())
				print(z)
			Roomba.Move(0,0)
		except KeyboardInterrupt:
			break
	start = goal
	while True: #Loop that asks for initial x and y coordinates
		try:
			x_final = float(input("X axis coordinate:"))
			y_final = float(input("Y axis coordinate:"))
			break
		except ValueError:
			print("Please input a number")
			continue
	goal = (x_final,y_final)
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