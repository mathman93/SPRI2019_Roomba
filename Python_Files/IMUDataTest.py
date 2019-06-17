''' IMUDataTest.py
Purpose: Read and save data from IMU while Roomba is moving
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
GPIO.setup(yled, GPIO.OUT, initial=GPIO.HIGH)
# IMU Setup
imu = RoombaCI_lib.LSM9DS1_I2C()
# Add code here to calibrate IMU
start_time = time.time()
while time.time()-start_time<20:
	Roomba.Move(0,50)
	mag_x, mag_y, mag_z = imu.magnetic
	print('Magnetometer (gauss): {0:0.5f},{1:0.5f},{2:0.5f}'.format(mag_x, mag_y, mag_z))
Roomba.Move(0,0)
imu.CalibrateGyro()
GPIO.setup(yled, GPIO.OUT, initial=GPIO.LOW)
GPIO.output(gled, GPIO.LOW)

# Main Code #
delay = 0.25 # time delay for data (in seconds)
delay_time = delay
base_time = time.time()
# At this point the loop will repeat until 'Ctrl+C' is typed.
while True:
	try:
		if time.time() - base_time > delay_time:
			# Read acceleration, magnetometer, gyroscope, and temperature data
			data_time = time.time() - base_time
			accel_x, accel_y, accel_z = imu.acceleration
			mag_x, mag_y, mag_z = imu.magnetic
			gyro_x, gyro_y, gyro_z = imu.gyro
			temp = imu.temperature
			
			# Print values
			print('Time: {0:0.6f}'.format(data_time))
			print('Acceleration (m/s^2): {0:0.5f},{1:0.5f},{2:0.5f}'.format(accel_x, accel_y, accel_z))
			print('Magnetometer (gauss): {0:0.5f},{1:0.5f},{2:0.5f}'.format(mag_x, mag_y, mag_z))
			print('Gyroscope (degrees/sec): {0:0.5f},{1:0.5f},{2:0.5f}'.format(gyro_x, gyro_y, gyro_z))
			print('Temperature: {0:0.3f}C'.format(temp))
			
			# Add delay for next iteration
			delay_time += delay

	except KeyboardInterrupt:
		break

## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.ShutDown() # Shutdown Roomba serial connection
GPIO.cleanup() # Reset GPIO pins for next program