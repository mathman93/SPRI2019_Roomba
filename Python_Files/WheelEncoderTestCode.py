''' TestCode.py
Purpose: Simple test of running Python code
Last Modified: 6/7/2019
'''

## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO
import RoombaCI_lib
import os.path

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
# Open a text file for data retrieval
file_name_input = input("Name for data file: ")
dir_path = "/home/pi/SPRI2019_Roomba/Data_Files/" # Directory path to save file
file_name = os.path.join(dir_path, file_name_input+".txt") # text file extension
file = open(file_name, "w") # Open a text file for storing data
	# Will overwrite anything that was in the text file previously

start_time = time.time()

Roomba.StartQueryStream(43,44)

l = [(100,0),(0,50),(-100,0),(0,-25),(75,0),(0,-50),(100,0)]

for i in range(len(l)):
	data_time = time.time()
	Roomba.Move(l[i][0],l[i][1])
	while time.time() - start_time <=4.0:
		if Roomba.Available()>0:
			[left_encoder, right_encoder]=Roomba.ReadQueryStream(43,44)
			print("{0},{1},{2}".format(left_encoder, right_encoder,data_time))
			print("")
			file.write("{0},{1},{2}\n".format(left_encoder, right_encoder,data_time))
	start_time = time.time()
Roomba.Move(0,0)
Roomba.PauseQueryStream()
if Roomba.Available()>0:
	y = Roomba.DirectRead(Roomba.Available())
	print(y)
file.close()
## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.ShutDown() # Shutdown Roomba serial connection
GPIO.cleanup() # Reset GPIO pins for next program