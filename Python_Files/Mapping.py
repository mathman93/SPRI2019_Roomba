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


## Functions and Definitions ##
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
		x_pos = (id[0]*200)+100
		y_pos = (id[1]*200)+100
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
			new_cost = cost_so_far[current]+distance(current,next)
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

	path.append(start)
	path.reverse()
	return path


def removePointFromWorld(x,y,MyWorld):
	NewWorld = MyWorld
	list = NewWorld.edges.pop((x,y))
	for p in list:
		NewWorld.edges[p].remove((x,y))

	NewWorld.points.remove((x,y))
	return NewWorld

## -- Code Starts Here -- ##
start = (0,0)
goal = (5,3)
MyWorld = makeworld(6,4)
path = A_star(start,goal,MyWorld)
print(path)
for point in path:
	location = MyWorld.Location(point)
	print("Moving to point {0}".format(location))

MyWorld = removePointFromWorld(2,1,MyWorld)

path = A_star(start,goal,MyWorld)

for point in path:
	location = MyWorld.Location(point)
	print("Moving to point {0}".format(location))

#Print Stuff
print(path)
for point in MyWorld.edges.keys():
	value = MyWorld.edges[point]
	print("{0}:{1}".format(point,value))

#print(MyWorld.neighbors((10,1)))
#print(MyWorld.neighbors((5,1)))
#print(MyWorld.Location((5,2)))

## -- Ending Code Starts Here -- ##

