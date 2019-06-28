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

# Variables and Constants
backup_time = 2.0 # Amount of time spent backing up
f = 0 # Forward/Backward speed
s = 0 # Rotation Speed
bump_time = time.time() - 2.0 # Assures that the roomba doesn't start in backup mode
wheel_diameter = 72
counts_per_rev = 508.8
distance_between_wheels = 235
C_theta = (wheel_diameter*math.pi)/(counts_per_rev*distance_between_wheels)
distance_per_count = (wheel_diameter*math.pi)/counts_per_rev
data_time = time.time()

bump_mode = False

#file.write("{0},{1},{2},{3},{4},{5}\n".format(0,left_start, right_start,x_position,y_position,theta))
Roomba.StartQueryStream(7)

while True:
	try:
		if Roomba.Available()>0:
			data_time2 = time.time()
			# Get bump value, light bumper value, then get left and right encoder values and find the change in each
			bump = Roomba.ReadQueryStream(7)

			if(bump%4) > 0: # If the roomba bumps into something...
				bump_time = time.time() #Sets up timer
				bump_code = (bump%4) #Will tell if left/right/center bump
				bump_mode = True # Keeps in memory that the roomba will now try and track the object it bumped into
			if time.time() - bump_time < backup_time: # If hasn't backed up for long enough...
				if bump_code == 1: # If bump right...
					f = -100
					s = -50
				if bump_code == 2 or bump_code == 3: # If bump left or center...
					f = -100
					s = 50
			elif bump_mode: # If not having to back up but still has bumped into something before...
				if bump_code == 1:
					f = 100
					s = 15
				if bump_code == 2 or bump_code == 3:
					f = 100
					s = -15
			else:
				f = 100
				s = 0

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