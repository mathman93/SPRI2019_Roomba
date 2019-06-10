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
LED_variables = [time.time(),time.time(),time.time(),False,False,False]

## Functions and Definitions ##
''' Displays current date and time to the screen
	'''
def DisplayDateTime():
	# Month day, Year, Hour:Minute:Seconds
	date_time = time.strftime("%B %d, %Y, %H:%M:%S", time.gmtime())
	print("Program run: ", date_time)

def LEDflash(Switch_time,bool,start_time,stop_time,led):
	if stop_time-start_time >=Switch_time:
		bool = not bool
		if bool:
			GPIO.output(led,GPIO.HIGH)
		else:
			GPIO.output(led,GPIO.LOW)
		start_time = start_time+Switch_time
	return start_time,bool

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
while True:
	try:
		stop_time = time.time()
		LED_variables(0),LED_variables(3) = LEDflash(0.7,LED_variables(3),LED_variables(0),stop_time,rled)
		LED_variables(1),LED_variables(4) = LEDflash(1.1,LED_variables(4),LED_variables(1),stop_time,yled)
		LED_variables(2),LED_variables(5) = LEDflash(1.5,LED_variables(5),LED_variables(2),stop_time,gled)
	except KeyboardInterrupt:
		break



	


## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.ShutDown() # Shutdown Roomba serial connection
GPIO.cleanup() # Reset GPIO pins for next program
