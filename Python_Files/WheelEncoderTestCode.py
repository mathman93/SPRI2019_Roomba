''' WheelEncoderTestCode.py
Purpose: Read wheel encoder values and calculate trajecotry
Last Modified: 6/12/2019
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

dict = {0:[100,0,10],
	1:[0,100,5],
	2:[50,50,5]}



y_pos = 0
x_pos= 0
theta = 0
counter=0
data_time = time.time()
for i in range(len(dict.keys())):
	[f,s,t] = dict[i]
	Roomba.Move(f,s)
	while time.time() - start_time <=t:
		if Roomba.Available()>0:
			data_time2 = time.time()
			[left_encoder, right_encoder]=Roomba.ReadQueryStream(43,44)
			counter += 1
			delta_l = left_encoder(counter)-left_encoder(counter-1)
			delta_r = right_encoder(counter)-right_encoder(counter-1)
			countdifference = delta_l-delta_r
			delta_theta = (delta_l-delta_r)*((72*180)/(508.8*235))
			theta += delta_theta
			if countdifference == 0:
				delta_d = 0.5*(delta_l+delta_r)*((72*pi)/508.8)
			else:
				delta_d = 2*(235(delta_l/(delta_l-delta_r)-.5)*sin(theta/2)
			x_pos += delta_d*cos(delta_theta-.5*theta)
			y_pos += delta_d*sin(delta_theta-.5*theta)
			print("{0},{1},{2},{3},{4},{5}".format(data_time2-data_time,left_encoder,right_encoder,x_pos,y_pos,theta))
			print("")
			file.write("{0},{1},{2},{3},{4},{5}\n".format(data_time2-data_time,left_encoder, right_encoder,x_pos,y_pos,theta))
	start_time = time.time()
Roomba.Move(0,0)
Roomba.PauseQueryStream()
if Roomba.Available()>0:
	z = Roomba.DirectRead(Roomba.Available())
	print(z)
file.close()
## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.ShutDown() # Shutdown Roomba serial connection
GPIO.cleanup() # Reset GPIO pins for next program