# Main video loop

import cv2
import numpy as np
import videoutils as vu


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

roll = 0
pitch = 0
filter = 0.6
one_minus_filter = 1-filter

# function to get roll and pitch (in radians)
def get_roll_pitch():
    accel = mpu.get_acceleration()    
    pitch = filter*pitch + one_minus_filter*asin(accel[0]/sqrt(accel[0]*accel[0] + accel[1]*accel[1]+ accel[2]*accel[2] ))
    roll = filter*roll + one_minus_filter*atan(accel[1]/accel[2])
    return roll, pitch


####################################################################
# this is simple class to calculate frames per second
####################################################################
fps = vu.FPS()



####################################################################
# Setup the camera
####################################################################

sensor = vu.sensor_IMX219 # select the PiCamera V2.1 sensor

camera = vu.configure_camera(sensor, lens_f=2.1, image_mode=5, frame_rate=15)

# Create a VideoCapture object and read from input file
# If the input is the camera, pass 0 instead of the video file name
cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)  # Set horizontal resolution
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)  # Set vertical resolution
cap.set(cv2.CAP_PROP_FPS, 10)

# Check if camera opened successfully
if (cap.isOpened()== False): 
  print("Error opening video stream or file")

fps.start()
roll, pitch = get_roll_pitch()

while(1):

  # Capture frame-by-frame
  ret, frame = cap.read()
  if ret == True:

    ##############################################
    # instructions that must always run 
    ##############################################
    # get the instantaneous roll/pitch
    roll, pitch = get_roll_pitch()

    # calc the pixel shift
    dph, dpw = vu.get_pixel_shift(roll, pitch, camera)

    ##############################################
    # State machine for vidoe modes 
    ##############################################

    shift_frame = vu.image_tranlate(frame, dph, dpw)
    
    # Display the resulting frame
    cv2.imshow('Frame',shift_frame)

    fps.update()

    # Press Q on keyboard to  exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break

  # Break the loop
  else: 
    break
fps.stop()

print('Frames per second: ',fps.fps())

# When everything done, release the video capture object
cap.release()

# Closes all the frames
cv2.destroyAllWindows()
