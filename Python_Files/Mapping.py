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
class GridWorld():
	def __init__(self):
		self.edges = {}
		self.points = []
	def neighbors(self,id):
		return self.edges.get(id,[])
	def Location(self,id):
		x_pos = (id[0]*200)+100
		y_pos = (id[1]*200)+100




## -- Code Starts Here -- ##
MyWorld = GridWorld()

for x in range(6):
	for y in range(4):
		MyWorld.points.append((x,y))

for point in MyWorld.points:
	x = point[0]
	y = point[1]
	neighbors = [(x+1,y+1),(x+1,y),(x+1,y-1),(x,y-1),(x,y+1),(x-1,y-1),(x-1,y),(x-1,y+1)]
	group = []
	for point1 in neighbors:
		if point1 in MyWorld.points:
			group.append(point1)
	MyWorld.edges[point] = group


for point in MyWorld.edges.keys:
	value = MyWorld.edges[point]
	print("{0}:{1}".format(point,value))





## -- Ending Code Starts Here -- ##

