''' FiveDOF_Test.py
Purpose: Read and save data from IMU while Roomba is moving
Also calculate inertial force vector (direction of "up") from IMU data
Last Modified: 6/18/2019
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

# Dictionary of move commands
dict = {0:[0,0,2],
	1:[100,0,5],
	2:[0,0,2],
	3:[0,50,5],
	4:[0,0,2],
	5:[50,-50,5],
	6:[50,50,5],
	7:[0,0,2]
	}

# Roomba Constants
wheel_diameter = 72
counts_per_rev = 508.8
distance_between_wheels = 235
C_theta = (wheel_diameter*math.pi)/(counts_per_rev*distance_between_wheels)
distance_per_count = (wheel_diameter*math.pi)/counts_per_rev

# Get initial wheel encoder values
[left_start,right_start]=Roomba.Query(43,44)
start_time = time.time()
data_time = time.time()
data_time_init = time.time() - data_time
# Read initial acceleration, magnetometer, gyroscope, and temperature data
accel_x, accel_y, accel_z = imu.acceleration
mag_x, mag_y, mag_z = imu.magnetic
gyro_x, gyro_y, gyro_z = imu.gyro
temp = imu.temperature

accel_length = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)

R_est = (1/accel_length) * np.array([accel_x, accel_y, accel_z])
# Split up vector for easy reading
R_estX = R_est[0]
R_estY = R_est[1]
R_estZ = R_est[2]
gyro_init = [gyro_x, gyro_y, gyro_z]

# Variables and Constants
y_position = 0 # Position of Roomba along y-axis (in mm)
x_position = 0 # Position of Roomba along x-axis (in mm)
theta = 0 # Heading of Roomba (in radians)

# Print values
print('Time: {0:0.6f}'.format(data_time_init))
print('Acceleration (m/s^2): {0:0.5f},{1:0.5f},{2:0.5f}'.format(accel_x, accel_y, accel_z))
print('Magnetometer (gauss): {0:0.5f},{1:0.5f},{2:0.5f}'.format(mag_x, mag_y, mag_z))
print('Gyroscope (degrees/sec): {0:0.5f},{1:0.5f},{2:0.5f}'.format(gyro_x, gyro_y, gyro_z))
print('Temperature: {0:0.3f}C'.format(temp))
# Print the left encoder, right encoder, x position, y position, and theta
print('L/R Wheel Encoders (counts): {0},{1}'.format(left_start,right_start))
print('Roomba X/Y Position (mm): {0:.3f},{1:.3f}'.format(x_position,y_position))
print('Roomba Orientation (radians): {0:.6f}'.format(theta))
# Print estimated inertial force values
print('Estimated Force Vector: {0:0.5f},{1:0.5f},{2:0.5f}'.format(R_estX, R_estY, R_estZ))
# Write IMU data, wheel encoder data, and estimated inertial force vector values to a file.
file.write("{0:0.6f},{1:0.5f},{2:0.5f},{3:0.5f},{4:0.5f},{5:0.5f},{6:0.5f},{7:0.5f},{8:0.5f},{9:0.5f},{10},{11},{12:0.5f},{13:0.5f},{14:0.5f}\n"\
	.format(data_time_init, accel_x, accel_y, accel_z,mag_x, mag_y, mag_z,gyro_x, gyro_y, gyro_z, left_encoder, right_encoder, R_estX, R_estY, R_estZ))
			
Roomba.StartQueryStream(43,44)

for i in range(len(dict.keys())):
	# Get peices of dictionary and tell the roomba to move
	[f,s,t] = dict[i]
	Roomba.Move(f,s)
	while time.time() - start_time <=t:
		# If data is available
		if Roomba.Available()>0:
			data_time2 = time.time() - data_time
			delta_time = data_time2 - data_time_init
			# Get left and right encoder values and find the change in each
			[left_encoder, right_encoder]=Roomba.ReadQueryStream(43,44)
			# Read acceleration, magnetometer, gyroscope, and temperature data
			accel_x, accel_y, accel_z = imu.acceleration
			mag_x, mag_y, mag_z = imu.magnetic
			gyro_x, gyro_y, gyro_z = imu.gyro
			temp = imu.temperature
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

			# Calculate inertial force vector (i.e., direction of "up")
			# Get readings from the accelerometer
			R_acc = np.array([accel_x, accel_y, accel_z])
			R_acc_length = np.linalg.norm(R_acc)
			R_acc = (1/R_acc_length) * R_acc # Normalize acceleration values
			# Calculate updated angles of the force vector using the gyroscope
			theta_xz = math.atan2(R_est[0],R_est[2]) + (math.radians(0.5*(gyro_y+gyro_init[1]))*delta_time)
			theta_yz = math.atan2(R_est[1],R_est[2]) + (math.radians(0.5*(gyro_x+gyro_init[0]))*delta_time)

			R_gyro = np.zeros(R_acc.shape)
			R_gyro[0] = math.sin(theta_xz)/math.sqrt(1 + (math.cos(theta_xz)*math.tan(theta_yz))**2)
			R_gyro[1] = math.sin(theta_yz)/math.sqrt(1 + (math.cos(theta_yz)*math.tan(theta_xz))**2)
			R_gyro[2] = math.sqrt(1 - R_gyro[0]**2 - R_gyro[1]**2) * np.sign(R_est[2])
			
			w_gyro = 10 # gyro weight value in range [5, 20]
			
			R_est = (R_acc + (w_gyro * R_gyro))/(1 + w_gyro) # New estimate of inertial force vector
			R_est_length = np.linalg.norm(R_est)
			R_est = (1/R_est_length) * R_est # Normalize estimated values
			# Split up vector for easy reading
			R_estX = R_est[0]
			R_estY = R_est[1]
			R_estZ = R_est[2]
			
			# Print values
			print('Time: {0:0.6f}'.format(data_time2))
			print('Acceleration (m/s^2): {0:0.5f},{1:0.5f},{2:0.5f}'.format(accel_x, accel_y, accel_z))
			print('Magnetometer (gauss): {0:0.5f},{1:0.5f},{2:0.5f}'.format(mag_x, mag_y, mag_z))
			print('Gyroscope (degrees/sec): {0:0.5f},{1:0.5f},{2:0.5f}'.format(gyro_x, gyro_y, gyro_z))
			print('Temperature: {0:0.3f}C'.format(temp))
			# Print the left encoder, right encoder, x position, y position, and theta
			print('L/R Wheel Encoders (counts): {0},{1}'.format(left_encoder,right_encoder))
			print('Roomba X/Y Position (mm): {0:.3f},{1:.3f}'.format(x_position,y_position))
			print('Roomba Orientation (radians): {0:.6f}'.format(theta))
			# Print estimated inertial force values
			print('Estimated Force Vector: {0:0.5f},{1:0.5f},{2:0.5f}'.format(R_estX, R_estY, R_estZ))
			# Write IMU data, wheel encoder data, and estimated inertial force vector values to a file.
			file.write("{0:0.6f},{1:0.5f},{2:0.5f},{3:0.5f},{4:0.5f},{5:0.5f},{6:0.5f},{7:0.5f},{8:0.5f},{9:0.5f},{10},{11},{12:0.5f},{13:0.5f},{14:0.5f}\n"\
				.format(data_time2, accel_x, accel_y, accel_z,mag_x, mag_y, mag_z,gyro_x, gyro_y, gyro_z, left_encoder, right_encoder, R_estX, R_estY, R_estZ))
			# Save values for next iteration
			left_start = left_encoder
			right_start = right_encoder
			data_time_init = data_time2
			gyro_init = [gyro_x, gyro_y, gyro_z]
		# End if Roomba.Available()
	# End while time.time() - start_time <=t:
	start_time = time.time()
# End for i in range(len(dict.keys())):
Roomba.Move(0,0) # Stop Roomba
Roomba.PauseQueryStream() # Pause data stream
if Roomba.Available() > 0: # If anything is in the Roomba receive buffer
	z = Roomba.DirectRead(Roomba.Available()) # Clear out excess Roomba data
	#print(z) # Include for debugging
file.close() # Close data file
## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.ShutDown() # Shutdown Roomba serial connection
GPIO.cleanup() # Reset GPIO pins for next program
