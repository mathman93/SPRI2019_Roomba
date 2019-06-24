''' RotationTest.py
Purpose: Test to see what the initial orientation of the Roomba is from the IMU
	 magnetometer readings
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
file_name_input = input("Name for data file: ")
dir_path = "/home/pi/SPRI2019_Roomba/Data_Files/" # Directory path to save file
file_name = os.path.join(dir_path, file_name_input+".txt") # text file extension
file = open(file_name, "w") # Open a text file for storing data
	# Will overwrite anything that was in the text file previously

#while True:
	#try:
		#speed = int(input("Speed of rotation:"))
		#duration = float(input("Duration of rotation:"))
		#break
	#except ValueError:
		#print("Please input a number.")
		#continue

speed = 100
duration = 10

# Dictionary of move commands
dict = {0:[0,0,10],
	1:[0,speed, duration],
	2:[0,0,10]
	}

#Get initial IMU readings and then print them out
mag_x, mag_y, mag_z = imu.magnetic
print('Magnetometer (gauss): {0:0.5f},{1:0.5f},{2:0.5f}'.format(mag_x, mag_y, mag_z))

# Variables and Constants
y_position = 0 # Position of Roomba along y-axis (in mm)
x_position = 0 # Position of Roomba along x-axis (in mm)
accel_length = math.sqrt((accel_x**2)+(accel_y**2)+(accel_z**2)) # Distance of vector made by acceleration
r_accel_x = accel_x / accel_length #Normalized acceleration values
r_accel_y = accel_y / accel_length
r_accel_z = accel_z / accel_length
r_estimate_x = r_accel_x #Estimates of normalized values, in first iteration, estimate is just initial reading
r_estimate_y = r_accel_y
r_estimate_z = r_accel_z
init_gyro_x = gyro_x #Sets initial gyro readings for later usage
init_gyro_y = gyro_y
w_gyro = 15 #Constant used to weight measurements

k_current = [r_estimate_x, r_estimate_y, r_estimate_z] #Initial K vector in regards to body is set to initial normalized estimate vector
i_current = [1, 0, 0] #Initial I vector is set to standard vector unit length, but can be set to initial magnetometer readings if using magnetometer
k_i_product = [(DotProduct(k_current,i_current)*x) for x in k_current]
i_current = [(a-b) for a,b in zip(i_current, k_i_product)]
j_current = CrossProduct(k_current, i_current)
length_i = math.sqrt(DotProduct(i_current,i_current)) 
length_k = math.sqrt(DotProduct(k_current,k_current))
length_j = math.sqrt(DotProduct(j_current,j_current))
i_norm = [x / length_i for x in i_current] # Normalized values of I,K and J prime
k_norm = [x / length_k for x in k_current]
j_norm = [x / length_j for x in j_current]
theta = math.atan2(j_norm[0],i_norm[0]) # Heading of Roomba (in radians) as calculated by the wheel encoders
new_theta = math.atan2(j_norm[0],i_norm[0]) # Heading of Roomba (in radians) as calculated by the IMU
if new_theta < 0:
	new_theta += 2*math.pi

S_gyro = 10 # Weight of gyro
S_accel = 2 # Weight of acceleromater
S_mag = 2 #Weight of magnetometer

accel_sum = [0, 0, 0]
gyro_sum = [0, 0, 0]
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

file.write("{0:0.6f},{1:0.5f},{2:0.5f},{3:0.5f}\n".format(data_time2,mag_x,mag_y,mag_z))

for i in range(len(dict.keys())):
	# Get peices of dictionary and tell the roomba to move
	[f,s,t] = dict[i]
	Roomba.Move(f,s)
	while time.time() - start_time <=t:
		# If data is available
		if Roomba.Available()>0:
			data_time2 = time.time() - data_time

			mag_x, mag_y, mag_z = imu.magnetic
			readings_counter += 1 #Counts how many times readings have been gathered

			mag_list = [mag_x, mag_y, mag_z]
			mag_sum = [(a+b) for a,b in zip(mag_sum, mag_list)]
			mag_avg = [(x/readings_counter) for x in mag_sum]
			mag_x, mag_y, mag_z = mag_avg # Set magnetometer values that will be used later to be the average of two readings

			print('Time: {0:0.6f}'.format(data_time2))
			print('Magnetometer (gauss): {0:0.5f},{1:0.5f},{2:0.5f}'.format(mag_x, mag_y, mag_z))
			file.write("{0:0.6f},{1:0.5f},{2:0.5f},{3:0.5f}\n".format(data_time2,mag_x,mag_y,mag_z))
			
			readings_counter = 0 # Reset counter for averages next time around
			mag_sum = [0, 0, 0]
		else:
			mag_x, mag_y, mag_z = imu.magnetic
			readings_counter += 1
			mag_list = [mag_x,mag_y,mag_z]
			mag_sum = [(a+b) for a,b in zip(mag_sum, mag_list)]

		# End if Roomba.Available()
	# End while time.time() - start_time <=t:
	start_time = time.time()
# End for i in range(len(dict.keys())):
Roomba.Move(0,0) # Stop Roomba
if Roomba.Available() > 0: # If anything is in the Roomba receive buffer
	z = Roomba.DirectRead(Roomba.Available()) # Clear out excess Roomba data
	print(z) # Include for debugging
file.close() # Close data file
## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.ShutDown() # Shutdown Roomba serial connection
GPIO.cleanup() # Reset GPIO pins for next program
