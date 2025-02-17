''' BetterBumpCheck.py
Purpose: Go to designated coordinate position while using bumpers and light bumpers to navigate around obstacles
Last Modified: 6/17/2019
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
ba_time = 1.0 #Amount of time that the roomba will back up after bumping into an object
sp_time = 1.0 #Amount of time that the roomba will rotate after backing up after bumping into an object
bump_time = time.time() - (ba_time + sp_time) #Initializes the bump timer, make sure the code starts in "goal mode"
y_position = 0 #Initial y position that the roomba thinks it is at
x_position = 0 #Initial x position that the roomba thinks it is at
theta = 0 #Starting rotation (in radians) that the roomba thinks it stars at
distance = 0 #Total distance the roomba thinks it has travelled
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

distance_to_end = math.sqrt((x_final-x_position)**2 +(y_final-y_position)**2) #Calculates distance to endpoint at designated coordinates
theta_initial = math.atan2((y_final-y_position),(x_final-x_position)) #Angle of the line between the roomba and the endpoint from zero
theta_d = ((theta_initial-theta)%(2 * math.pi)) #How many radians the roomba must rotate before it is facing the endpoint
print("{0:.6f},{1},{2},{3:.3f},{4:.3f},{5:.6f},{6},{7}".format(0,left_start,right_start,x_position,y_position,theta,distance_to_end,theta_d))

Roomba.StartQueryStream(43,44,7,45) #Begins receiving data from the left and right wheel encoders, the bumper, and the light bumper

while True:
	try:
		# Tell the roomba to move
		while distance_to_end > 3:
			if Roomba.Available()>0:

				data_time2 = time.time()
				# Get bump value, light bumper value, left and right encoder values
				[bump, l_bump, left_encoder, right_encoder]=Roomba.ReadQueryStream(7,45,43,44)
				
				# Finds the change in the left and right wheel encoder values
				delta_l = left_encoder-left_start
				delta_r = right_encoder-right_start

				#Checks if the encoder values have rolled over, and if so, subtracts/adds accordingly to assure normal delta values
				if delta_l < -1*(2**15):
					delta_l += (2**16)
				elif delta_l > (2**15):
					delta_l -+ (2**16)
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

				#Conditional statements that calculate change in distance according to whether or not the roomba is going straight(delta_l - delta_r = 0)
				if delta_l-delta_r == 0:
					delta_d = 0.5*(delta_l+delta_r)*distance_per_count
					delta_distance = delta_d
				else:
					delta_d = 2*(235*(delta_l/(delta_l-delta_r)-.5))*math.sin(delta_theta/2)
					delta_distance = 235*(delta_l/(delta_l-delta_r)-.5) * delta_theta
				
				# Find new x and y position
				x_position = x_position + delta_d*math.cos(theta-.5*delta_theta)
				y_position = y_position + delta_d*math.sin(theta-.5*delta_theta)
				distance += delta_distance
				# Find distance to end and theta_initial
				distance_to_end = math.sqrt((x_final-x_position)**2 +(y_final-y_position)**2)
		
				theta_initial = math.atan2((y_final-y_position),(x_final-x_position))
				# Normalize what theta initial is to between 0-2pi
				if theta_initial <0:
					theta_initial += 2*math.pi
				# Calculate theta_d and normalize it to 0-2pi
				# This value is the difference between the direction the roomba is supposed to be going and the direction it is actually going
				theta_d = ((theta_initial-theta)%(2*math.pi))
				# get theta_d between -pi and pi
				if theta_d > math.pi:
					theta_d -= 2*math.pi

				#Variables indicating which light bumpers have been tripped
				l_l_bump = int(l_bump % 2)
				fl_l_bump = int((l_bump % 4) / 2)
				cl_l_bump = int((l_bump % 8) / 4)
				cr_l_bump = int((l_bump % 16) / 8)
				fr_l_bump = int((l_bump % 32) / 16)
				r_l_bump = int((l_bump % 64) / 32)				
				
				#Main code determining movement of the roomba				
				if(bump%4) > 0: #If a bump is detected...
					bump_time = time.time() #Sets up timer
					bump_code = (bump%4) #Will tell if left/right/center bump
				if time.time() - bump_time < ba_time: #If before backing up time is up...
					f = -40 #Back up
					s = 0
				elif time.time() - bump_time < (ba_time + sp_time): #If before spinning time is up...
					if bump_code == 1: #If bump right
						f = 0
						s = -50 #Spin counterclockwise
					if bump_code == 2: #If bump left
						f = 0
						s = 50 #Spin clockwise
					if bump_code == 3: #If bump center
						f = 0
						s = 100 #Spin clockwise faster
				else: #Gives movement commands based on light sensor detection
					if fl_l_bump or cl_l_bump or cr_l_bump or fr_l_bump: #If any center four light sensors are tripped...
						if cr_l_bump or fr_l_bump: #If one of the right sensors is tripped...
							f = 0
							s = -50 #Spin counterclockwise
						else:	#If one of the left sensors is tripped...
							f = 0
							s = 50 #Spin clockwise
					elif l_l_bump or r_l_bump: #If only the far left or far right sensors are tripped...
						f = 80 #Go forward
						s = 0
					else: #Gives movement if nothing is in the way
						if abs(theta_d) > (math.pi / 4): #If theta_d is greater than pi/4 radians...
							s_set = 100 # Spin faster
						elif abs(theta_d) > (math.pi / 36): #If theta_d is getting closer...
							s_set = 60 # Spin normal speed
						else: # otherwise, if theta_d is fairly small...
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
						else: #Roomba is facing the endpoint and go straight forward
							s = 0
						if theta_d > (math.pi / 2) or theta_d < (math.pi / -2): #If the end point is beyond 90 degrees in either direction, the roomba will rotate in place
							f = 0
						elif abs(2*radius*math.sin(theta_d)) > distance_to_end: #If the end point is within the circle that is drawn by the roomba's turn path... 
							f = 0 #The roomba will not move forward
						else:
							f = f_set
				Roomba.Move(f,s) #Makes the roomba move with the parameters given to it
				# Print and write the time, left encoder, right encoder, x position, y position, and theta
				print("{0:.6f},{1},{2},{3:.3f},{4:.3f},{5:.6f},{6},{7},{8}".format(data_time2-data_time,left_encoder,right_encoder,x_position,y_position,theta,distance_to_end,theta_d,distance))
				print("")
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

## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.ShutDown() # Shutdown Roomba serial connection
GPIO.cleanup() # Reset GPIO pins for next program