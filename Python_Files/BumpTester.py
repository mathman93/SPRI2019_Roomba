''' BumpTester.py
Purpose: Used to test how the bumpers detect collision at different angles. Also used to test BumpAngle function
Last Modified: 7/2/2019
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

''' Function that returns the angle of an object (in degrees in the range 0-360) that the roomba is bumping into.
	Uses the 'bumper' bumper reading (query code 7) and the 'l_bumper' light bumper reading (query code 45)
	'''
def BumpAngle(bumper, l_bumper):
	if l_bumper == 23:
		return 10
	elif l_bumper == 22:
		return 20
	elif l_bumper == 44 or l_bumper == 60:
		return 30
	elif l_bumper == 12 or l_bumper == 28:
		return 40
	elif l_bumper == 56:
		return 50
	elif l_bumper == 24:
		return 60
	elif l_bumper == 27:
		return 350
	elif l_bumper == 13:
		return 340
	elif l_bumper == 15:
		return 330
	elif l_bumper == 14:
		return 320
	elif l_bumper == 6:
		return 310
	elif l_bumper == 3 or l_bumper == 1:
		return 300

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

start_time = time.time()

# Variables and Constants

Roomba.StartQueryStream(7,45,46,47,48,49,50,51)

while True:
	data_time = time.time() - start_time
	[bump, l_bump, l_l_bump, fl_l_bump, cl_l_bump, cr_l_bump, fr_l_bump, r_l_bump]=Roomba.ReadQueryStream(7,45,46,47,48,49,50,51)
	if (bump%4) > 0:
		print("Time: {0:0.5f}\nBumper: {1}, Light Bumper: {2:0>6b}\nL L-Bumper:{3}, FL L-Bumper: {4}, CL L-Bumper: {5}, CR L-Bumper: {6}, FR L-Bumper: {7}, R L-Bumper: {8}\n".format(data_time,bump,l_bump,l_l_bump,fl_l_bump,cl_l_bump,cr_l_bump,fr_l_bump,r_l_bump))
		print("Angle of Object: {0}".format(BumpAngle(bump,l_bump)))
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