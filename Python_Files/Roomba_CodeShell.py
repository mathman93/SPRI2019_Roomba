''' Roomba_CodeShell.py
Purpose: Basic code for running Roomba, Xbee, and IMU
	Sets up Roomba, Xbee, and IMU and calibrates;
IMPORTANT: Must be run using Python 3 (python3)
Last Modified: 6/6/2018
'''
## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO
import RoombaCI_lib

## Variables and Constants ##
global Xbee # Specifies connection to Xbee
Xbee = serial.Serial('/dev/ttyUSB0', 115200) # Baud rate should be 115200
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

''' Set up IMU and (optionally) calibrate magnetometer and gyroscope
	Parameters:
		imu = instance of LSM9DS1_I2C class;
		Roomba = instance of Create_2 class;
		mag_cal = boolean; If magnetometer calibration is desired, set to True
		gyro_cal = boolean; If gyroscope calibration is desired, set to True
	'''
def IMUCalibration(imu, Roomba, mag_cal = True, gyro_cal = True):
	# Clear out first reading from all sensors (They can sometimes be bad)
	x = imu.magnetic
	x = imu.acceleration
	x = imu.gyro
	x = imu.temperature
	# Calibrate the magnetometer and the gyroscope
	if mag_cal:
		Roomba.Move(0,100) # Start the Roomba spinning
		imu.CalibrateMag() # Determine magnetometer offset values
		Roomba.Move(0,0) # Stop the Roomba
		time.sleep(0.1) # Wait for the Roomba to settle
		# Display offset values
		print("mx_offset = {:f}; my_offset = {:f}; mz_offset = {:f}"\
			.format(imu.m_offset[0], imu.m_offset[1], imu.m_offset[2]))
	else:
		print(" Skipping magnetometer calibration.")
	
	if gyro_cal:
		imu.CalibrateGyro() # Determine gyroscope offset values
		# Display offset values
		print("gx_offset = {:f}; gy_offset = {:f}; gz_offset = {:f}"\
			.format(imu.g_offset[0], imu.g_offset[1], imu.g_offset[2]))
	else:
		print(" Skipping gyroscope calibration")
	time.sleep(1.0) # Give some time to read the offset values

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
GPIO.output(yled, GPIO.HIGH) # Indicate within setup sequence
# Initialize IMU
print(" Starting IMU...")
imu = RoombaCI_lib.LSM9DS1_I2C() # Initialize IMU
time.sleep(0.1)
print(" Calibrating IMU...")
IMUCalibration(imu, Roomba, False, False)
print(" Calibration Complete")

GPIO.output(yled, GPIO.LOW) # Indicate IMU setup sequence is complete

if Xbee.inWaiting() > 0: # If anything is in the Xbee receive buffer
	x = Xbee.read(Xbee.inWaiting()).decode() # Clear out Xbee input buffer
	#print(x) # Include for debugging
GPIO.output(gled, GPIO.LOW)

# Main Code #


## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly

Roomba.ShutDown() # Shutdown Roomba serial connection
Xbee.close()
GPIO.cleanup() # Reset GPIO pins for next program
