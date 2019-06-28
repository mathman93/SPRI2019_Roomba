''' Dock_Test.py
Purpose: Read and save data from IMU while Roomba is moving
Also calculate Directional Cosine Matrix (DCM) to determine orientation
Last Modified: 6/28/2019
'''

## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO
import RoombaCI_lib
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

def BlinkLED(led):
	led = not led
	if rled_bool:
		GPIO.output(rled, GPIO.HIGH)
	else:
		GPIO.output(rled, GPIO.LOW)
	return led

## -- Code Starts Here -- ##
# Setup Code #
GPIO.setmode(GPIO.BCM) # Use BCM pin numbering for GPIO
DisplayDateTime() # Display current date and time

# LED Pin setup
GPIO.setup(yled, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(rled, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(gled, GPIO.OUT, initial=GPIO.LOW)

# Wake Up Roomba Sequence
GPIO.output(gled, GPIO.HIGH) # Turn on green LED to say we are alive and setting up
print(" Starting ROOMBA...")
Roomba = RoombaCI_lib.Create_2("/dev/ttyS0", 115200)
Roomba.ddPin = 23 # Set Roomba dd pin number
GPIO.setup(Roomba.ddPin, GPIO.OUT, initial=GPIO.LOW)

Roomba.DirectWrite(128) # From off, start Roomba OI (sets to Passive)
time.sleep(1.0)
Roomba.DirectWrite(131) # From Passive mode, send to Safe Mode
time.sleep(0.1)
Roomba.BlinkCleanLight() # Test if Roomba is in Safe Mode
if Roomba.Available() > 0: # If anything is in the Roomba receive buffer
	x = Roomba.DirectRead(Roomba.Available()) # Clear out Roomba boot-up info
	print(x) # Include for debugging

#StartUp(Roomba, 23, 131) # Start up Roomba in Safe mode
print(" ROOMBA Setup Complete")

GPIO.output(gled, GPIO.LOW) # Turn off green LED to say we have finished setup sequence

# Main Code #
print(" Now Undocking...")
backup_base = time.time()
blink_base = backup_base
rled_bool = False

Roomba.Move(-40,0)
while time.time() - backup_base < 12.5:
	if time.time() - blink_base > 0.5:
		rled_bool = BlinkLED(rled_bool)
		blink_base += 0.5
	# End if
# End while
	
Roomba.Move(0,0)
time.sleep(0.5)

print(" Now Docking...")
charging_state = 0
blink_base = time.time()
Roomba.StartQueryStream(21)
Roomba.Dock()
while chargin_state == 0:
	if Roomba.Available() > 0:
		charging_state = ReadQueryStream(21)
	if time.time() - blink_base > 0.5:
		rled_bool = BlinkLED(rled_bool)
		blink_base += 0.5
# End while
Roomba.PauseQueryStream()
if Roomba.Available() > 0: # If anything is in the Roomba receive buffer
	x = Roomba.DirectRead(Roomba.Available()) # Clear out Roomba boot-up info
	print(x) # Include for debugging
time.sleep(5.0)

Roomba.PlaySMB() # For fun :)
## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.ShutDown() # Shutdown Roomba serial connection
GPIO.cleanup() # Reset GPIO pins for next program