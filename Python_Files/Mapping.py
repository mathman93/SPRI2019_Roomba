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

## Variables and Constants ##


## Functions and Definitions ##
# defines a world, finds the neighbors of certain points, and the location of thos points
class GridWorld():
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




## -- Code Starts Here -- ##
# Call the class gridworld
MyWorld = GridWorld()

# Create points that exist in the world
for x in range(6):
	for y in range(4):
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

#Print Stuff
#for point in MyWorld.edges.keys():
#	value = MyWorld.edges[point]
#	print("{0}:{1}".format(point,value))

#print(MyWorld.neighbors((10,1)))
#print(MyWorld.neighbors((5,1)))
#print(MyWorld.Location((5,2)))

## -- Ending Code Starts Here -- ##

