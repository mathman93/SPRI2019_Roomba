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
time1 = time.time()
while True:	
	try:
		time2 = time.time()
		if time2-time1 > .5:
			[light_bumper,ll_bumper,lfl_bumper,lcl_bumper,rcl_bumper,rfl_bumper,rl_bumper] = Roomba.Query(45,46,47,48,49,50,51)
			print ("{0:0>8b}".format(light_bumper))
			print ("ll_bumper:{0}".format(ll_bumper))
			print ("lfl_bumper:{0}".format(lfl_bumper))
			print ("lcl_bumper:{0}".format(lcl_bumper))
			print ("rcl_bumper:{0}".format(rcl_bumper))
			print ("rfl_bumper:{0}".format(rfl_bumper))
			print ("rl_bumper:{0}".format(rl_bumper))
			time1 = time1+.5
	except KeyboardInterrupt:
		break


## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.ShutDown() # Shutdown Roomba serial connection
GPIO.cleanup() # Reset GPIO pins for next program