''' BumpTester.py
Purpose: Used to test how the bumpers detect collision at different angles. Also used to test BumpAngle function
Last Modified: 7/8/2019
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
	Uses the 'bumper' bumper reading (query code 7) and the 'l_bumper' light bumper reading (query code 45). Should at least be accurate to a range of 20 degrees
	'''
'''
def BumpAngle(bumper, l_bumper):
	if bumper == 1 or bumper == 3: # If bumped on right or center...
		if l_bumper == 23 or l_bumper == 15 or (l_bumper == 20 and bumper == 3): # If (L, FL, CL, FR) or (FL) or (CL, FR, center)
			return 10
		elif l_bumper == 22 or (l_bumper == 20 and bumper == 3) or (l_bumper == 16 and bumper == 3) or (l_bumper == 4 and bumper == 3): # If (FL, CL, FR) or (CL, FR and center) or (FR and center) or (CL and center)
			return 20
		elif l_bumper == 44 or l_bumper == 60 or l_bumper == 20: # If (FL, CL, R) or (FL, CL, FR, R) or (CL, FR)
			return 30
		elif l_bumper == 12 or l_bumper == 28 or l_bumper == 40 or l_bumper == 36 or l_bumper == 4: # If (CL, CR) or (CL, CR, FR) or (R, CR) or (R, CL) or (CL)
			return 40
		elif l_bumper == 56 or l_bumper == 40 or l_bumper == 8: # If (CR, FR, R) or (CR)
			return 50
		elif l_bumper == 24 or l_bumper == 40: # If (CR, FR) or (CR, R)
			return 60
		elif l_bumper == 32: # If R light sensor triggered...
			return 70
		elif l_bumper == 0: # If no light sensor triggered...
			return 80
		else:
			return 0
	elif bumper == 2: # If bumped on left...
		if l_bumper == 27 or l_bumper == 9: # If CL, CR, FR light sensors triggered...
			return 350
		elif l_bumper == 13: # If L, CL, CR light sensors triggered...
			return 340
		elif l_bumper == 15: # If L, FL, CL, CR light sensors triggered...
			return 330
		elif l_bumper == 14: # If FL, CL, CR light sensors triggered...
			return 320
		elif l_bumper == 6: # If FL, CL light sensors triggered...
			return 310
		elif l_bumper == 3 or l_bumper == 1: # If L, FL or L sensors triggered...
			return 300
		elif l_bumper == 0: # If no light sensor triggered...
			return 290
		else:
			return 0
	else:
		return None
	'''

def BumpAngle(bumper,l_bumper)
	for i in range(6) #For all the possible binary digits that represent light bumpers being activated...
		if l_bumper & math.pow(2,i) == math.pow(2,i): #If the light bumper value indicates that the "i" light bumper is being triggered...
			l_bumper_list[i] = True # The placeholder boolean for that light bumper is set to true
		else: # If the "i" light bumper is not being triggered...
			l_bumper_list[i] = False # The placeholder boolean is set to false
	[L,FL,CL,CR,FR,R] = l_bumper_list # Sets the booleans to be used in conditionals to their own variables for easy reference
	if bumper == 3: # If roomba detects a bump in the center...
		return 0
	if bumper == 1: # If the roomba detects a bump on the right...
		if R: # If the far right light bumper is triggered at all...
			return 70
		elif not CR: # If the center right light bumper is not triggered at all...
			return 20
		else:
			return 45
	if bumper == 2: # If roomba detects a bump on the left...
		if not (L or FL or CL or CR): # If any of the left side or center right light bumpers are not triggered...
			return -70
		elif CR: # If the center right light bumper is triggered...
			return -20
		else: 
			return -45
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