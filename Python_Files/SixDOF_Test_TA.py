''' SixDOF_Test.py
Purpose: Read and save data from IMU while Roomba is moving
Also calculate Directional Cosine Matrix (DCM) to determine orientation
Last Modified: 6/21/2019
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
#x = imu.temperature
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
imu_file_name_input = input("Name for (IMU) data file: ")
dir_path = "/home/pi/SPRI2019_Roomba/Data_Files/" # Directory path to save file
imu_file_name = os.path.join(dir_path, imu_file_name_input+".txt") # text file extension
imu_file = open(imu_file_name, "w") # Open a text file for storing data
	# Will overwrite anything that was in the text file previously
dcm_file_name_input = input("Name for (DCM) data file: ")
dir_path = "/home/pi/SPRI2019_Roomba/Data_Files/" # Directory path to save file
dcm_file_name = os.path.join(dir_path, dcm_file_name_input+".txt") # text file extension
dcm_file = open(dcm_file_name, "w") # Open a text file for storing data
	# Will overwrite anything that was in the text file previously

# Dictionary of move commands
dict = {0:[0,0,20],
	1:[0,75,20],
	2:[0,0,1]
	}

# Roomba Constants
wheel_diameter = 72
counts_per_rev = 508.8
distance_between_wheels = 235
C_theta = (wheel_diameter*math.pi)/(counts_per_rev*distance_between_wheels)
distance_per_count = (wheel_diameter*math.pi)/counts_per_rev

accel_sum = np.zeros(3) # Vector of sum of accelerometer values
mag_sum = np.zeros(3) # Vector of sum of magnetometer values
omega_sum = np.zeros(3) # Vector of sum of gyroscope values
#temp_sum = 0 # Sum of temperature values
imu_counter = 0 # Number of summed values

# Read initial acceleration, magnetometer, gyroscope, and temperature data
for i in range(10): # Get 10 values for initial data
	accel_x, accel_y, accel_z = imu.acceleration
	mag_x, mag_y, mag_z = imu.magnetic
	gyro_x, gyro_y, gyro_z = imu.gyro
	#temp_raw = imu.temperature
	imu_counter += 1 # Incremenet counter
	accel_sum += np.array([accel_x, accel_y, accel_z])
	mag_sum += np.array([mag_x, mag_y, mag_z])
	omega_sum += np.array([gyro_x, gyro_y, gyro_z])
	#temp_sum += temp_raw
# End for loop
# Average summed values to get each initial reading
accel = accel_sum/imu_counter
mag = mag_sum/imu_counter
omega = omega_sum/imu_counter
#temp = temp_sum/imu_counter

accel_length = np.linalg.norm(accel) # Length of accelerometer vector

R_est = (1/accel_length) * accel # Normalized accelerometer values
K_B = R_est # Initial estimate of zenith versor
# Use only one of the following...
#I_B_init = np.array([1, 0, 0]) # Initial estimate of forward versor (w/out mag)
I_B_init = (mag*np.array([-1,1,1]))/np.linalg.norm(mag) # Estimate of forward versor (with mag)
# Must flip x-component due to magnetometer orientation on IMU
# Orthogonalize I_B_init, then normalize
I_B = I_B_init - (K_B.dot(I_B_init)*K_B)
I_B_length = np.linalg.norm(I_B)
I_B = (1/I_B_length) * I_B
J_B = np.cross(K_B, I_B) # Left Hand Rule
# Form DCM from component values
DCM_G = np.stack((I_B, J_B, K_B))
theta_imu = np.arctan2(J_B[0], I_B[0]) # Initial estimate of heading from IMU
if theta_imu < 0:
	theta_imu += 2*np.pi

# Variables and Constants
y_position = 0 # Position of Roomba along y-axis (in mm)
x_position = 0 # Position of Roomba along x-axis (in mm)
#theta = 0 # Heading of Roomba (in radians)
theta = theta_imu # Heading of Roomba (using imu)
#theta = math.atan2(-mag[1],-mag[0]) # Heading of Roomba (using magnetometer)

gyro_init = omega # Store initial values for next iteration

# Get initial wheel encoder values
[left_start,right_start]=Roomba.Query(43,44)
data_time = time.time()
data_time_init = time.time() - data_time

# Print values
print('Time: {0:0.6f}'.format(data_time_init))
print('Acceleration (m/s^2): {0:0.5f},{1:0.5f},{2:0.5f}'.format(accel[0], accel[1], accel[2]))
print('Magnetometer (gauss): {0:0.5f},{1:0.5f},{2:0.5f}'.format(mag[0], mag[1], mag[2]))
print('Gyroscope (degrees/sec): {0:0.5f},{1:0.5f},{2:0.5f}'.format(omega[0], omega[1], omega[2]))
#print('Temperature: {0:0.3f}C'.format(temp))
# Print the left encoder, right encoder, x position, y position, and theta
print('L/R Wheel Encoders (counts): {0},{1}'.format(left_start,right_start))
print('Roomba X/Y Position (mm): {0:.3f},{1:.3f}'.format(x_position,y_position))
print('Roomba Orientation (radians): {0:0.6f}, {1:0.6f}'.format(theta, theta_imu))
# Print DCM values [I_B; J_B; K_B]
print('DCM: [[{0:0.5f}, {1:0.5f}, {2:0.5f}]'.format(DCM_G[0,0], DCM_G[0,1], DCM_G[0,2]))
print('	[{0:0.5f}, {1:0.5f}, {2:0.5f}]'.format(DCM_G[1,0], DCM_G[1,1], DCM_G[1,2]))
print('	[{0:0.5f}, {1:0.5f}, {2:0.5f}]]'.format(DCM_G[2,0], DCM_G[2,1], DCM_G[2,2]))
# Write IMU data, wheel encoder data, and estimated inertial force vector values to a file.
imu_file.write("{0:0.6f},{1:0.5f},{2:0.5f},{3:0.5f},{4:0.5f},{5:0.5f},{6:0.5f},{7:0.5f},{8:0.5f},{9:0.5f},{10},{11},{12:0.6f},{13:0.6f}\n"\
	.format(data_time_init,accel[0],accel[1],accel[2],mag[0],mag[1],mag[2],omega[0],omega[1],omega[2],left_start,right_start,theta,theta_imu))
dcm_file.write("{0:0.5f},{1:0.5f},{2:0.5f},{3:0.5f},{4:0.5f},{5:0.5f},{6:0.5f},{7:0.5f},{8:0.5f}\n"\
	.format(DCM_G[0,0],DCM_G[0,1],DCM_G[0,2],DCM_G[1,0],DCM_G[1,1],DCM_G[1,2],DCM_G[2,0],DCM_G[2,1],DCM_G[2,2]))
Roomba.StartQueryStream(43,44)

for i in range(len(dict.keys())):
	# Get pieces of dictionary and tell the roomba to move
	start_time = time.time()
	[f,s,t] = dict[i]
	Roomba.Move(f,s)
	while time.time() - start_time <= t:
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
			#temp_raw = imu.temperature
			imu_counter += 1 # Increment counter
			# Accumulate IMU readings
			accel_sum += np.array([accel_x, accel_y, accel_z])
			mag_sum += np.array([mag_x, mag_y, mag_z])
			omega_sum += np.array([gyro_x, gyro_y, gyro_z])
			#temp_sum += temp_raw
			# Average summed values to get each initial reading
			accel = accel_sum/imu_counter
			mag = mag_sum/imu_counter
			omega = omega_sum/imu_counter
			#temp = temp_sum/imu_counter
			#omega *= 1.00472 # experimentally determined scale factor for gyro values

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
			R_acc = accel
			R_acc_length = np.linalg.norm(R_acc)
			R_acc = (1/R_acc_length) * R_acc # Normalize acceleration values
			# Calculate updated angles of the force vector using the gyroscope
			theta_xz = math.atan2(R_est[0],R_est[2]) + (np.radians(0.5*(omega[1]+gyro_init[1]))*delta_time)
			theta_yz = math.atan2(R_est[1],R_est[2]) + (np.radians(0.5*(omega[0]+gyro_init[0]))*delta_time)
			# Calculate estimate of inertial force vector from gyroscope data
			R_gyro = np.zeros(R_acc.shape)
			R_gyro[0] = math.sin(theta_xz)/math.sqrt(1 + (math.cos(theta_xz)*math.tan(theta_yz))**2)
			R_gyro[1] = math.sin(theta_yz)/math.sqrt(1 + (math.cos(theta_yz)*math.tan(theta_xz))**2)
			R_gyro[2] = np.sqrt(1 - R_gyro[0]**2 - R_gyro[1]**2) * np.sign(R_est[2])
			
			w_acc = 1 # Accelerometer weight value
			w_gyro = 10 # Gyroscope weight value
			
			R_est = ((w_acc * R_acc) + (w_gyro * R_gyro))/(w_acc + w_gyro) # New estimate of inertial force vector
			R_est_length = np.linalg.norm(R_est)
			R_est = (1/R_est_length) * R_est # Normalize estimated values
			K_A = R_est # New estimate of zenith vector from accelerometer (and gyro) data
			
			I_M = (mag*np.array([-1,1,1]))/np.linalg.norm(mag) # Estimate of "north" from magnetometer data
			# Must flip x-component due to magnetometer orientation on IMU
			# Calculate rotation change, delta_theta
			delta_theta_gyro = np.radians(omega) * delta_time # Gyro estimate of rotation
			delta_theta_gyro_para = K_B.dot(delta_theta_gyro) * K_B # Component parallel to K_B
			delta_theta_gyro_perp = delta_theta_gyro - delta_theta_gyro_para # Component perpendicular to K_B
			delta_theta_acc = np.cross(K_B, (K_A - K_B)) # Accelerometer estimate of rotation
			delta_theta_mag = np.cross(I_B, (I_M - I_B)) # Magnetometer estimate of rotation
			delta_theta_mag_para = K_B.dot(delta_theta_mag) * K_B # Component parallel to K_B
			delta_theta_mag_perp = delta_theta_mag - delta_theta_mag_para # Component perpendictular to K_B
			s_acc = 1 # Accelerometer weight value
			s_gyro = 10 # Gyroscope weight value
			s_mag = 1 # Magnetometer weight value
			# Component values of delta_theta
			delta_theta_perp = ((s_gyro*delta_theta_gyro_perp) + (s_acc*delta_theta_acc) + (s_mag*delta_theta_mag_perp))/(s_gyro + s_acc + s_mag)
			delta_theta_para = ((s_gyro*delta_theta_gyro_para) + (s_mag*delta_theta_mag_para))/(s_gyro + s_mag)
			delta_theta = delta_theta_perp + delta_theta_para # Combined values
			# Update versors
			K_B += np.cross(delta_theta, K_B)
			I_B += np.cross(delta_theta, I_B)
			# Orthogonalize updated versors
			error = 0.5*K_B.dot(I_B)
			K_B_prime = K_B - (error*I_B)
			I_B_prime = I_B - (error*K_B)
			J_B_prime = np.cross(K_B_prime, I_B_prime) # Left Hand Rule
			# Normalize updated versors
			K_B = K_B_prime/np.linalg.norm(K_B_prime)
			I_B = I_B_prime/np.linalg.norm(I_B_prime)
			J_B = J_B_prime/np.linalg.norm(J_B_prime)
			R_est = K_B # Reset R_est for next iteration
			# Form DCM from component values
			DCM_G = np.stack((I_B, J_B, K_B))
			theta_imu = np.arctan2(J_B[0], I_B[0]) # Initial estimate of heading from IMU
			if theta_imu < 0:
				theta_imu += 2*np.pi
			
			# Print values
			print('Time: {0:0.6f}'.format(data_time2))
			print('Acceleration (m/s^2): {0:0.5f},{1:0.5f},{2:0.5f}'.format(accel[0], accel[1], accel[2]))
			print('Magnetometer (gauss): {0:0.5f},{1:0.5f},{2:0.5f}'.format(mag[0], mag[1], mag[2]))
			print('Gyroscope (degrees/sec): {0:0.5f},{1:0.5f},{2:0.5f}'.format(omega[0], omega[1], omega[2]))
			#print('Temperature: {0:0.3f}C'.format(temp))
			#print('Data Counter: {0}'.format(imu_counter)) # Include for testing
			# Print the left encoder, right encoder, x position, y position, and theta
			#print('L/R Wheel Encoders (counts): {0},{1}'.format(left_encoder,right_encoder))
			#print('Roomba X/Y Position (mm): {0:.3f},{1:.3f}'.format(x_position,y_position))
			print('Roomba Orientation (radians): {0:0.6f}, {1:0.6f}'.format(theta, theta_imu))
			# Print DCM values [I_B; J_B; K_B]
			print('DCM: [[{0:0.5f}, {1:0.5f}, {2:0.5f}]'.format(DCM_G[0,0], DCM_G[0,1], DCM_G[0,2]))
			print('	[{0:0.5f}, {1:0.5f}, {2:0.5f}]'.format(DCM_G[1,0], DCM_G[1,1], DCM_G[1,2]))
			print('	[{0:0.5f}, {1:0.5f}, {2:0.5f}]]'.format(DCM_G[2,0], DCM_G[2,1], DCM_G[2,2]))
			# Delta_theta Debug Data
			#print('Delta_Accel: {0}'.format(delta_theta_acc))
			#print('Delta_Gyro: {0}'.format(delta_theta_gyro_para))
			#print('Delta_Mag: {0}'.format(delta_theta_mag_para))
			# Write IMU data, wheel encoder data to a file.
			imu_file.write("{0:0.6f},{1:0.5f},{2:0.5f},{3:0.5f},{4:0.5f},{5:0.5f},{6:0.5f},{7:0.5f},{8:0.5f},{9:0.5f},{10},{11},{12:0.6f},{13:0.6f}\n"\
				.format(data_time_init,accel[0],accel[1],accel[2],mag[0],mag[1],mag[2],omega[0],omega[1],omega[2],left_start,right_start,theta,theta_imu))
			dcm_file.write("{0:0.5f},{1:0.5f},{2:0.5f},{3:0.5f},{4:0.5f},{5:0.5f},{6:0.5f},{7:0.5f},{8:0.5f}\n"\
				.format(DCM_G[0,0],DCM_G[0,1],DCM_G[0,2],DCM_G[1,0],DCM_G[1,1],DCM_G[1,2],DCM_G[2,0],DCM_G[2,1],DCM_G[2,2]))
			# Save values for next iteration
			left_start = left_encoder
			right_start = right_encoder
			data_time_init = data_time2
			gyro_init = omega
			# Reset sums for next iteration
			accel_sum = np.zeros(3) # Vector of sum of accelerometer values
			mag_sum = np.zeros(3) # Vector of sum of magnetometer values
			omega_sum = np.zeros(3) # Vector of sum of gyroscope values
			#temp_sum = 0 # Sum of temperature values
			imu_counter = 0 # Number of summed values
		else: # If Roomba data hasn't come in
			# Read acceleration, magnetometer, gyroscope, and temperature data
			accel_x, accel_y, accel_z = imu.acceleration
			mag_x, mag_y, mag_z = imu.magnetic
			gyro_x, gyro_y, gyro_z = imu.gyro
			#temp_raw = imu.temperature
			imu_counter += 1 # Increment counter
			# Accumulate IMU readings
			accel_sum += np.array([accel_x, accel_y, accel_z])
			mag_sum += np.array([mag_x, mag_y, mag_z])
			omega_sum += np.array([gyro_x, gyro_y, gyro_z])
			#temp_sum += temp_raw

		# End if Roomba.Available()
	# End while time.time() - start_time <=t:
# End for i in range(len(dict.keys())):
Roomba.Move(0,0) # Stop Roomba
Roomba.PauseQueryStream() # Pause data stream
if Roomba.Available() > 0: # If anything is in the Roomba receive buffer
	z = Roomba.DirectRead(Roomba.Available()) # Clear out excess Roomba data
	print(z) # Include for debugging
imu_file.close() # Close data file
#dcm_file.close() # Close data file
Roomba.PlaySMB() # For fun :)
## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.ShutDown() # Shutdown Roomba serial connection
GPIO.cleanup() # Reset GPIO pins for next program
