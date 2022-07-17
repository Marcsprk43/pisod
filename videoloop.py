# Main video loop

import cv2
import numpy as np
from imutils.video.pivideostream import PiVideoStream
import videoutils as vu
from pymavlink import mavutil
import argparse
import time





####################################################################
# Video system states
####################################################################
ST_RAW_VIDEO = 0
ST_STABILIZE_VIDEO = 1
ST_FIND_TARGET = 2


####################################################################
# Set up the accelerometer for roll/pitch calcs
####################################################################
from mpu6050.MPU6050 import MPU6050
from math import sqrt, asin, atan, degrees

i2c_bus = 1
device_address = 0x68
x_accel_offset = -0
y_accel_offset = -0
z_accel_offset = 0
x_gyro_offset = 181
y_gyro_offset = 32
z_gyro_offset = 6
enable_debug_output = False

mpu = MPU6050(i2c_bus, device_address, x_accel_offset, y_accel_offset,
              z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset,
              sample_rate_divider=1, dlpf=0x04,
              a_debug=enable_debug_output)

filtered_roll = 0
filtered_pitch = 0
filter = 0.4
one_minus_filter = 1-filter

# function to get roll and pitch (in radians)
def get_roll_pitch():
    global filtered_roll, filtered_pitch
    accel = mpu.get_acceleration()    
    filtered_pitch = filter*filtered_pitch + one_minus_filter*asin(accel[0]/sqrt(accel[0]*accel[0] + accel[1]*accel[1]+ accel[2]*accel[2] ))
    filtered_roll = filter*filtered_roll + one_minus_filter*atan(accel[1]/accel[2])
    return filtered_roll, filtered_pitch


# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-p", "--picamera", type=int, default=1,
    help="whether or not the Raspberry Pi camera should be used")
args = vars(ap.parse_args())



####################################################################
# this is simple class to calculate frames per second
####################################################################

fps = vu.FPS()

####################################################################
# Setup the camera
####################################################################
resolution = (720,576)

sensor = vu.sensor_IMX219 # select the PiCamera V2.1 sensor

camera = vu.configure_camera(sensor, lens_f=2.1, image_mode=5, frame_rate=32)

# initialize the video stream and allow the cammera sensor to warmup
vs = PiVideoStream(resolution=resolution, framerate=30).start()

time.sleep(1)

frame = vs.read()

print('frame size:', frame.shape)


"""# Create a VideoCapture object and read from input file
# If the input is the camera, pass 0 instead of the video file name
cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)  # Set horizontal resolution
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)  # Set vertical resolution
cap.set(cv2.CAP_PROP_FPS, 30)

# Check if camera opened successfully
if (cap.isOpened()== False): 
  print("Error opening video stream or file")
"""
# initialize the FPS tracker
fps.start()
# initialize the roll pitch
roll, pitch = get_roll_pitch()


mode =  ST_STABILIZE_VIDEO
osd_overlay = 'Screen1'

##############################################
# Mavlink
##############################################

# Start a connection 
the_connection = mavutil.mavlink_connection('/dev/ttyAMA0', 57600)

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

mv = vu.Mavlink()

##############################################
##############################################
# Main loop
##############################################
##############################################

while(1):

  # Capture frame-by-frame
  frame = vs.read()
  #frame = imutils.resize(frame, width=800)

  ##############################################
  # instructions that must always run 
  ##############################################
  # get the instantaneous roll/pitch
  roll, pitch = get_roll_pitch()

  try: 
      the_connection.recv_match(blocking=False)
      mv.data['Lat'] = the_connection.messages['AHRS2'].lat  # Note, you can access message fields as attributes!
      mv.data['Lon'] = the_connection.messages['AHRS2'].lng  # Note, you can access message fields as attributes!
      mv.data['Altitude'] = the_connection.messages['AHRS2'].altitude  # Note, you can access message fields as attributes!
      mv.data['BattV'] = the_connection.messages['SYS_STATUS'].voltage_battery  # Note, you can access message fields as attributes!
      mv.data['BattPercent'] = the_connection.messages['SYS_STATUS'].battery_remaining
      mv.data['FlightMode'] = the_connection.messages['MAV_MODE'].
  except Exception as e:
      print(e, 'not received yet')

  ##############################################
  # State machine for video modes 
  ##############################################

  if mode == ST_RAW_VIDEO:
    # set the final frame to frame
    vu.apply_osd(frame, osd_overlay, mv)

  elif mode == ST_STABILIZE_VIDEO:
    # calc the pixel shift
    dph, dpw = vu.get_pixel_shift(roll, pitch, camera, factor=.75)

    frame = vu.image_tranlate(frame, dph, dpw)

    # always do this last
    vu.apply_osd(frame, osd_overlay, mv)

  elif mode == ST_FIND_TARGET:
    # calc the pixel shift
    dph, dpw = vu.get_pixel_shift(roll, pitch, camera)
    # stabilize the frame
    frame = vu.image_tranlate(frame, dph, dpw)

      ### do other stuff here

    
  # Display the resulting frame
  cv2.imshow('Frame',frame)

  fps.update()

  # Press Q on keyboard to  exit
  if cv2.waitKey(1) & 0xFF == ord('q'):
    break


# print the fps   
fps.stop()
print('Frames per second: ',fps.fps())

# When everything done, release the video capture object
vs.stop()

# Closes all the frames
cv2.destroyAllWindows()

print(the_connection.messages)
