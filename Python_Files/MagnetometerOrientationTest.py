''' RotationTest.py
Purpose: Test to see what the orientation of the Roomba is based on the wheel encoders vs the orientation
	 based on the magnetometer
Last Modified: 6/24/2019
'''

## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO
import RoombaCI_lib
import os.path
import math
import numpy as np

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

''' Returns 3 value list of the cross product of 3 value lists "a" and "b"
	'''
def CrossProduct(a,b):
	s0 = (a[1]*b[2]) - (a[2]*b[1])
	s1 = (a[2]*b[0]) - (a[0]*b[2])
	s2 = (a[0]*b[1]) - (a[1]*b[0])
	return [s0, s1, s2]

''' Returns single integer dot product of 3 value lists "a" and "b"
	'''
def DotProduct(a,b):
	return ((a[0]*b[0]) + (a[1] * b[1]) + (a[2]*b[2]))

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
GPIO.output(yled, GPIO.HIGH)
print(" Starting IMU...")
imu = RoombaCI_lib.LSM9DS1_I2C()
time.sleep(0.1)
# Clear out first reading from all sensors (They can sometimes be bad)
x = imu.magnetic
x = imu.acceleration
x = imu.gyro
# Calibrate the magnetometer and the gyroscope
print(" Calibrating IMU...")
Roomba.Move(0,100) # Start the Roomba spinning
imu.CalibrateMag() # Determine magnetometer offset values
Roomba.Move(0,0) # Stop the Roomba
time.sleep(0.1) # Wait for the Roomba to settle
imu.CalibrateGyro() # Determine gyroscope offset values
# Display offset values
print("mx_offset = {:f}; my_offset = {:f}; mz_offset = {:f}"\
	.format(imu.m_offset[0], imu.m_offset[1], imu.m_offset[2]))
print("gx_offset = {:f}; gy_offset = {:f}; gz_offset = {:f}"\
	.format(imu.g_offset[0], imu.g_offset[1], imu.g_offset[2]))
print(" Calibration Complete")
time.sleep(1.0) # Give some time to read the offset values
GPIO.output(gled, GPIO.LOW)
GPIO.output(yled, GPIO.LOW)

# Main Code #

# Open a text file for data retrieval
#file_name_input = input("Name for data file: ")
#dir_path = "/home/pi/SPRI2019_Roomba/Data_Files/" # Directory path to save file
#file_name = os.path.join(dir_path, file_name_input+".txt") # text file extension
#file = open(file_name, "w") # Open a text file for storing data
	# Will overwrite anything that was in the text file previously

# Get initial wheel encoder values
[left_start,right_start]=Roomba.Query(43,44)

#Get initial IMU readings and then print them out
mag_x, mag_y, mag_z = imu.magnetic
print('Magnetometer (gauss): {0:0.5f},{1:0.5f},{2:0.5f}'.format(mag_x, mag_y, mag_z))

# Variables and Constants
y_position = 0 # Position of Roomba along y-axis (in mm)
x_position = 0 # Position of Roomba along x-axis (in mm)
theta = math.atan2(mag_y,mag_x) # Heading of Roomba (in radians) as calculated by the wheel encoders
if theta < 0:
	theta += 2*math.pi
print('Wheel Encoder Heading (radians): {0:0.5f}'.format(theta))
mag_theta = math.atan2(mag_y,mag_x) #Heading of Roomba(in radians) as calculated by the magnetometer
if mag_theta < 0:
	mag_theta += 2*math.pi

mag_sum = [0, 0, 0]
readings_counter = 0

# Roomba Constants
wheel_diameter = 72
counts_per_rev = 508.8
distance_between_wheels = 235
C_theta = (wheel_diameter*math.pi)/(counts_per_rev*distance_between_wheels)
distance_per_count = (wheel_diameter*math.pi)/counts_per_rev

start_time = time.time()
data_time = time.time()
data_time_init = time.time() - data_time

#file.write("{0:0.6f},{1:0.5f},{2:0.5f},{3:0.5f},{4:0.5f}\n".format(data_time_init,mag_x,mag_y,mag_z,theta))

theta_initial = 0
theta_d = theta_initial-mag_theta
if theta_d < 0:
	theta_d += 2*math.pi

Roomba.StartQueryStream(43,44)

while True:
	try:
		# If data is available
		if Roomba.Available()>0:
			data_time2 = time.time() - data_time
			# Get left and right encoder values and find the change in each
			[left_encoder, right_encoder]=Roomba.ReadQueryStream(43,44)
			mag_x, mag_y, mag_z = imu.magnetic
			readings_counter += 1 #Counts how many times readings have been gathered

			mag_list = [mag_x, mag_y, mag_z]
			mag_sum = [(a+b) for a,b in zip(mag_sum, mag_list)]
			mag_avg = [(x/readings_counter) for x in mag_sum]
			mag_x, mag_y, mag_z = mag_avg # Set magnetometer values that will be used later to be the average of two readings

			mag_theta = math.atan2(mag_y,mag_x)
			if mag_theta < 0:
				mag_theta += 2*math.pi

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
				# Determine what method to use to find the change in distance
			if delta_l-delta_r == 0:
				delta_d = 0.5*(delta_l+delta_r)*distance_per_count
			else:
				delta_d = 2*(235*(delta_l/(delta_l-delta_r)-.5))*math.sin(delta_theta/2)
			
			# Find new x and y position
			x_position = x_position + delta_d*math.cos(theta-.5*delta_theta)
			y_position = y_position + delta_d*math.sin(theta-.5*delta_theta)

			# Calculate theta_d and normalize it to 0-2pi
			# This value is the difference between the direction were supposed to be going and the direction we are going
			theta_d = ((theta_initial-mag_theta)%(2*math.pi))
			# get theta_d between -pi and pi
			if theta_d > math.pi:
				theta_d -= 2*math.pi

			if abs(theta_d) > (math.pi / 4): #If theta_d is greater than pi/4 radians...
				s_set = 100 # Spin faster
			elif abs(theta_d) > (math.pi / 36): #If theta_d is getting closer...
				s_set = 60 # Spin normal speed
			else: # otherwise, if theta_d is fairly small
				s_set = 20 # Spin slow

			if theta_d > 0.5: #Rotates clockwise if theta_d is positive
				s = s_set
			elif theta_d < -0.5: #Rotates counterclockwise if theta_d is negative
				s = s_set * -1
			else:
				s = 0
			
			Roomba.Move(0,s)

			print('Time: {0:0.6f}'.format(data_time2))
			print('Magnetometer (gauss): {0:0.5f},{1:0.5f},{2:0.5f}'.format(mag_x, mag_y, mag_z))
			print('Wheel Encoder Heading (radians): {0:0.5f}'.format(theta))
			print('Magnetometer Heading (radians): {0:0.5f}'.format(mag_theta))
			#file.write("{0:0.6f},{1:0.5f},{2:0.5f},{3:0.5f},{4:0.5f},{5:0.5f}\n".format(data_time2,mag_x,mag_y,mag_z,theta,mag_theta))
			
			readings_counter = 0 # Reset counter for averages next time around
			mag_sum = [0, 0, 0]
			left_start = left_encoder
			right_start = right_encoder
			readings_counter = 0
		else:
			mag_x, mag_y, mag_z = imu.magnetic
			readings_counter += 1
			mag_list = [mag_x,mag_y,mag_z]
			mag_sum = [(a+b) for a,b in zip(mag_sum, mag_list)]
		# End if Roomba.Available()
		# End while time.time() - start_time <=t:
	except KeyboardInterrupt:
		break
# End for i in range(len(dict.keys())):
Roomba.Move(0,0) # Stop Roomba
Roomba.PauseQueryStream() # Pause data stream
if Roomba.Available() > 0: # If anything is in the Roomba receive buffer
	z = Roomba.DirectRead(Roomba.Available()) # Clear out excess Roomba data
	print(z) # Include for debugging
#file.close() # Close data file
## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.ShutDown() # Shutdown Roomba serial connection
GPIO.cleanup() # Reset GPIO pins for next program
