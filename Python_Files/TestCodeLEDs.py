''' TestCode.py
Purpose: Simple test of running Python code
Last Modified: 6/7/2019
'''

## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO
import RoombaCI_lib

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

# Main Code #
r_Switch_time=0.7
y_Switch_time=1.1
g_Switch_time=1.5
r_bool = False
y_bool = False
g_bool = False
start_time_r = time.time()
start_time_y = time.time()
start_time_g = time.time()
while True:
	try:
		stop_time = time.time()
		if stop_time-start_time_r >=r_Switch_time:
			r_bool = not r_bool
			if r_bool==True:
				GPIO.output(rled,GPIO.LOW)
			else:
				GPIO.output(rled,GPIO.HIGH)
			start_time_r = start_time_r+r_Switch_time
		if stop_time-start_time_y >=y_Switch_time:
			y_bool = not y_bool
			if y_bool==True:
				GPIO.output(yled,GPIO.LOW)
			else:
				GPIO.output(yled,GPIO.HIGH)
			start_time_y = start_time_y+y_Switch_time
		if stop_time-start_time_g >=g_Switch_time:
			g_bool = not g_bool
			if g_bool==True:
				GPIO.output(gled,GPIO.LOW)
			else:
				GPIO.output(gled,GPIO.HIGH)
			start_time_g = start_time_g+g_Switch_time
	except KeyboardInterupt:
		break



## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.ShutDown() # Shutdown Roomba serial connection
GPIO.cleanup() # Reset GPIO pins for next program
