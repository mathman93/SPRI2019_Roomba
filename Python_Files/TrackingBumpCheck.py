''' ChangeMoveOnFly.py
Purpose: Use bumpers to try and follow a wall or object continuously
Last Modified: 6/28/2019
'''

## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO
import RoombaCI_lib
import os.path
import math

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
# Main Code #
# Open a text file for data retrieval
#file_name_input = input("Name for data file: ")
#dir_path = "/home/pi/SPRI2019_Roomba/Data_Files/" # Directory path to save file
#file_name = os.path.join(dir_path, file_name_input+".txt") # text file extension
#file = open(file_name, "w") # Open a text file for storing data
	# Will overwrite anything that was in the text file previously

start_time = time.time()
[left_start,right_start]=Roomba.Query(43,44)
# Variables and Constants
backup_time = 1.0 # Amount of time spent backing up
corner_time = 1.5 # Amount of time that it takes before the roomba starts turning more sharply (makes sure it turns around corners)
f = 0 # Forward/Backward speed
s = 0 # Rotation Speed
bump_time = time.time() - 2.0 # Assures that the roomba doesn't start in backup mode
bump_count =0
bump_track = 0 # Keeps track of how many times the roomba has bumped into a wall
y_position = 0
x_position = 0
theta = 0
wheel_diameter = 72
counts_per_rev = 508.8
distance_between_wheels = 235
C_theta = (wheel_diameter*math.pi)/(counts_per_rev*distance_between_wheels)
distance_per_count = (wheel_diameter*math.pi)/counts_per_rev
data_time = time.time()

bump_mode = False # Used to tell whether or not the roomba has bumped into something and is supposed to be "tracking"

#file.write("{0},{1},{2},{3},{4},{5}\n".format(0,left_start, right_start,x_position,y_position,theta))
Roomba.StartQueryStream(7,43,44) # Start getting bumper values

while True:
	try:
		if Roomba.Available()>0:
			data_time2 = time.time()
			# Get bump value
			[bump,left_encoder, right_encoder] = Roomba.ReadQueryStream(7,43,44)
			delta_l = left_encoder-left_start
			delta_r = right_encoder-right_start
			# Determine the change in theta and what that is currently
			delta_theta = (delta_l-delta_r)*C_theta
			theta += delta_theta
			# Determine what method to use to find the change in distance
			if delta_l-delta_r == 0:
				delta_d = 0.5*(delta_l+delta_r)*distance_per_count
			else:
				delta_d = 2*(235*(delta_l/(delta_l-delta_r)-.5))*math.sin(delta_theta/2)
			# Find new x and y position
			x_position = x_position + delta_d*math.cos(theta-.5*delta_theta)
			y_position = y_position + delta_d*math.sin(theta-.5*delta_theta)
			if(bump%4) > 0: # If the roomba bumps into something...
				bump_time = time.time() #Sets up timer
				bump_mode = True # Keeps in memory that the roomba will now try and track the object it bumped into
				bump_count += 1 # Increases whenever the roomba bumps into something
				if bump_count < 2:
					bump_code = (bump%4) #Will tell if left/right/center bump
			if time.time() - bump_time < backup_time and bump_count < 2: # If hasn't backed up for long enough and it's the first bump...
				if bump_code == 1: # If bump right...
					f = -50 #Back up, spin counterclockwise
					s = -150
				if bump_code == 2 or bump_code == 3: # If bump left or center...
					f = -50 #Back up, spin clockwise
					s = 150
			elif time.time() - bump_time < (backup_time / 2): # If hasn't backed up for long enough and is not the first bump...
				if bump_code == 1: # If bump right...
					f = -50 # Back up, spin counterclockwise slower
					s = -50
				if bump_code == 2 or bump_code == 3: # If bump left or center...
					f = -50 #Back up, spin clockwise slower
					s = 50
			elif bump_mode and time.time() - bump_time < (backup_time + corner_time): # If not having to back up but still has bumped into something before...
				if bump_code == 1: # If bump right...
					f = 100 # Go forward, turn clockwise
					s = 15
				if bump_code == 2 or bump_code == 3: # If bump left...
					f = 100 # Go forward, turn counterclockwise
					s = -15
			elif bump_mode: # If not having to back up and bumped into something, but been a while...
				if bump_code == 1: # If bump right...
					f = 100 # Go forward, turn hard clockwise
					s = 100
				if bump_code == 2 or bump_code == 3: # If bump left...
					f = 100 # Go forward, turn hard counterclockwise
					s = -100
			else: # If haven't bumped into anything left...
				f = 100 # Go straight forward
				s = 0
			Roomba.Move(f,s) # Move with given forward and spin values
			print("{0},{1},{2},{3},{4},{5}".format(data_time2-data_time,left_encoder,right_encoder,x_position,y_position,theta))
			left_start = left_encoder
			right_start = right_encoder
			
			#file.write("{0},{1},{2},{3},{4},{5}\n".format(data_time2-data_time,left_encoder, right_encoder,x_position,y_position,theta))
	except KeyboardInterrupt:
		break
Roomba.Move(0,0)
Roomba.PauseQueryStream()
if Roomba.Available()>0:
	z = Roomba.DirectRead(Roomba.Available())
	print(z)
time.sleep(0.1)
#file.close()
## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.ShutDown() # Shutdown Roomba serial connection
GPIO.cleanup() # Reset GPIO pins for next program