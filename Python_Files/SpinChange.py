''' ChangeMoveOnFly.py
Purpose: Change movement while robot is moving
Last Modified: 6/12/2019
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
y_position = 0
x_position = 0
theta = 0
wheel_diameter = 72
counts_per_rev = 508.8
distance_between_wheels = 235
C_theta = (wheel_diameter*math.pi)/(counts_per_rev*distance_between_wheels)
distance_per_count = (wheel_diameter*math.pi)/counts_per_rev
data_time = time.time()
while True: #Loop that asks for initial x and y coordinates
	try:
		x_final = float(input("x position:"))
		y_final = float(input("y position:"))
		break
	except ValueError:
		print("Please input a number")
		continue
distance_to_end = math.sqrt((x_final-x_position)**2 +(y_final-y_position)**2)
theta_initial = math.atan2((y_final-y_position),(x_final-x_position))
theta_d = theta_initial-theta
print("{0:.6f},{1},{2},{3:.3f},{4:.3f},{5:.6f},{6},{7}".format(0,left_start,right_start,x_position,y_position,theta,distance_to_end,theta_d))

#file.write("{0},{1},{2},{3},{4},{5}\n".format(0,left_start, right_start,x_position,y_position,theta))
Roomba.StartQueryStream(43,44)

while True:
	try:
		# Tell the roomba to move
		while distance_to_end > 3:
			if Roomba.Available()>0:

				data_time2 = time.time()
				# Get left and right encoder values and find the change in each
				[left_encoder, right_encoder]=Roomba.ReadQueryStream(43,44)
				delta_l = left_encoder-left_start
				if delta_l < -1*(2**15): #Checks if the encoder values have rolled over, and if so, subtracts/adds accordingly to assure normal delta values
					delta_l += (2**16)
				elif delta_l > (2**15):
					delta_l -+ (2**16)
				delta_r = right_encoder-right_start
				if delta_r < -1*(2**15):
					delta_r += (2**16)
				elif delta_r > (2**15):
					delta_r -+ (2**16)
				# Determine the change in theta and what that is currently
				delta_theta = (delta_l-delta_r)*C_theta
				theta += delta_theta
				# If theta great than 2pi subtract 2pi and vice versus. Normalize theta to 0-2pi to show what my heading is.
				if theta >= 2*math.pi:
					theta -= 2*math.pi
				elif theta < 0:
					theta += 2*math.pi
				# Determine what method to use to find the change in distance
				if delta_l-delta_r == 0:		
					delta_d = 0.5*(delta_l+delta_r)*distance_per_count
				else:
					delta_d = 2*(235*(delta_l/(delta_l-delta_r)-.5))*math.sin(delta_theta/2)
				# Find new x and y position
				x_position = x_position + delta_d*math.cos(theta-.5*delta_theta)
				y_position = y_position + delta_d*math.sin(theta-.5*delta_theta)
				# Find distance to end and theta_initial
				distance_to_end = math.sqrt((x_final-x_position)**2 +(y_final-y_position)**2)
		
				theta_initial = math.atan2((y_final-y_position),(x_final-x_position))
				# Normalize what theta initial is to between 0-2pi
				if theta_initial <0:
					theta_initial += 2*math.pi
				# Calculate theta_d and normalize it to 0-2pi
				# This value is the difference between the direction were supposed to be going and the direction we are going
				theta_d = ((theta_initial-theta)%(2*math.pi))
				# get theta_d between -pi and pi
				if theta_d > math.pi:
					theta_d -= 2*math.pi

				if abs(theta_d) > (math.pi / 4): #If theta_d is greater than pi/4 radians...
					s_set = 100 # Spin faster
				elif abs(theta_d) > (math.pi / 36): #If theta_d is getting closer...
					s_set = 60 # Spin normal speed
				else: # otherwise, if theta_d is fairly small
					s_set = 20 # Spin slow
				if distance_to_end > 150: #If distance_to_end is greater than 150 mm...
					f_set = 120 #Go faster
				elif distance_to_end > 50: # If distance_to_end is greater than 50 mm...
					f_set = 80 #Go fast
				else: #otherwise, if distance_to_end is less than 50 mm...
					f_set = 40 #Go slow

				radius = ((235 / 2) * (f_set / s_set)) #Radius of circle of the roomba's turn for the given f_set and s_set values

				if theta_d > 0: #Rotates clockwise if theta_d is positive
					s = s_set
				elif theta_d < 0: #Rotates counterclockwise if theta_d is negative
					s = s_set * -1
				else:
					s = 0
				if theta_d > (math.pi / 2) or theta_d < (math.pi / -2): #If the end point is beyond 90 degrees in either direction, the roomba will rotate in place
					f = 0
				elif abs(2*radius*math.sin(theta_d)) > distance_to_end: #If the end point is within the circle that is drawn by the roomba's turn path, then the roomba will rotate in place 
					f = 0
				else:
					f = f_set
				Roomba.Move(f,s) #Makes the roomba move with the parameters given to
				# Print and write the time, left encoder, right encoder, x position, y position, and theta
				print("{0:.6f},{1},{2},{3:.3f},{4:.3f},{5:.6f},{6},{7}".format(data_time2-data_time,left_encoder,right_encoder,x_position,y_position,theta,distance_to_end,theta_d))
				print("")
				#file.write("{0},{1},{2},{3},{4},{5}\n".format(data_time2-data_time,left_encoder, right_encoder,x_position,y_position,theta))
				left_start = left_encoder
				right_start = right_encoder
		Roomba.PauseQueryStream() #Pauses the query stream while new coordinates are being input
		if Roomba.Available()>0:
			z = Roomba.DirectRead(Roomba.Available())
			print(z)
		Roomba.Move(0,0)
		while True: #Loop that asks the user for another set of x and y coordinates for the roomba to go to
			try:
				x_final = float(input("x position:"))
				y_final = float(input("y position:"))
				break
			except ValueError: #Prints the message if anything but a number is input, then re asks for th coordinates
				print("Please enter a number")
				continue
		distance_to_end = math.sqrt((x_final-x_position)**2 +(y_final-y_position)**2) #Recalculates distance_to_end before the main loop starts
		Roomba.ResumeQueryStream() #Resumes the query stream to continue as the roomba moves again
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