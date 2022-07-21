# Main video loop
import cv2
import numpy as np
from imutils.video.pivideostream import PiVideoStream
import videoutils as vu
from pymavlink import mavutil
import time
import math

####################################################################
# Video system states
####################################################################
ST_RAW_VIDEO = 0
ST_STABILIZE_VIDEO = 1
ST_FIND_TARGET = 2
####################################################################
# Set up the accelerometer for roll/pitch calcs
####################################################################
"""
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

def get_roll_pitch():
    global filtered_roll, filtered_pitch
    accel = mpu.get_acceleration()    
    filtered_pitch = filter*filtered_pitch + one_minus_filter*asin(accel[0]/sqrt(accel[0]*accel[0] + accel[1]*accel[1]+ accel[2]*accel[2] ))
    filtered_roll = filter*filtered_roll + one_minus_filter*atan(accel[1]/accel[2])
    return filtered_roll, filtered_pitch

"""

roll = 0.0
pitch = 0.0


####################################################################
# this is simple class to calculate frames per second
####################################################################

fps = vu.FPS()

####################################################################
# Setup the camera
####################################################################

resolution = (720,576)
resolution = (1640,1232)
framerate = 20
image_number = 0 

sensor = vu.sensor_IMX219 # select the PiCamera V2.1 sensor

camera = vu.configure_camera(sensor, lens_f=2.1, image_mode=5, frame_rate=15)

# initialize the video stream and allow the cammera sensor to warmup
vs = PiVideoStream(resolution=resolution, framerate=framerate).start()

time.sleep(1)

frame = vs.read()


print('frame size:', frame.shape)

# initialize the FPS tracker
fps.start()


mode =  ST_STABILIZE_VIDEO
osd_overlay = 'Screen1'

##############################################
# Mavlink
##############################################

# Start a connection 
the_connection = mavutil.mavlink_connection('/dev/serial0', baud=115200,
                                            dialect='ardupilotmega', autoreconnect=True)

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
print("Waiting for mavlink heartbeat")
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))
# Wait for the first heartbeat 

# Set up the message type frequencies
vu.request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1)
vu.request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, 30)
vu.request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 1)
vu.request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT  , 1)
vu.request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_POWER_STATUS  , 1)
vu.request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_MEMINFO  , 1)
vu.request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT  , 1)
vu.request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT  , 1)
vu.request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU  , 1)
vu.request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2  , 1)
vu.request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE  , 1)
vu.request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT  , 1)
vu.request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_SYSTEM_TIME  , 1)
vu.request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_AHRS  , 1)
vu.request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_HWSTATUS  , 1)
vu.request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_TERRAIN_REPORT  , 1)
vu.request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT  , 1)
vu.request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_VIBRATION  , 1)
vu.request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE  , 1)
vu.request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD  , 1)

# Initialize the OSD data dictionary
mv = vu.Mavlink()

home_alt = 260.
home_location = True     # flag that is set when the drone starts up

##############################################
##############################################
# Main loop
##############################################
##############################################

while(1):

  # Capture frame-by-frame
  frame = vs.read()

  ##############################################
  # instructions that must always run 
  ##############################################


  try:  # receive mavlink messages
    
    m = the_connection.recv_msg()
    
    msg_tries = 0
    while not (m is None) and msg_tries < 2:  # read in all the accumulated mavlink messages
      m = the_connection.recv_msg()
      msg_tries += 1

  except Exception as e:
      print('Cannot receive messages:', e)

  try:    # extract AHRS2 message
    mv.data['Lat'] = round(the_connection.messages['AHRS2'].lat/1e7,6) # Note, you can access message fields as attributes!
    mv.data['Lon'] = round(the_connection.messages['AHRS2'].lng.lat/1e7,6)  # Note, you can access message fields as attributes!
    mv.data['Altitude_MSL'] = the_connection.messages['AHRS2'].altitude  # Note, you can access message fields as attributes!
    mv.data['Yaw'] = the_connection.messages['AHRS2'].yaw  # Note, you can access message fields as attributes!

    mv.data['Altitude'] = mv.data['Altitude_MSL'] - home_alt

    print(mv.data['Altitude_MSL'], home_alt, mv.data['Altitude'], mv.data['BaseMode'])

    # save roll and pitch in global vars
    roll = the_connection.messages['AHRS2'].roll
    pitch = the_connection.messages['AHRS2'].pitch

  except Exception as e:
    print('AHRS2 message not recieved yet',e)
    roll = 0.0
    pitch = 0.0
  else: 
    # set the home alt if we are still in home location
    if ( home_location and not (mv.data['BaseMode'] & 128) ):
      home_alt = 0.9*home_alt + 0.1*mv.data['Altitude_MSL']     # filter the home alt readings
      print(home_alt)

  try:    # extract SYS_STATUS message
    mv.data['BattV'] = the_connection.messages['SYS_STATUS'].voltage_battery  # Note, you can access message fields as attributes!
    mv.data['BattPercent'] = the_connection.messages['SYS_STATUS'].battery_remaining
  except Exception as e:
    print('SYS_STATUS message not recieved yet',e)  

  try:   #extract the HEARTBEAT message
    mv.data['BaseMode'] = the_connection.messages['HEARTBEAT'].base_mode  # the base_mode describes the armed, disarmed, etc. status
    mv.data['FlightMode'] = the_connection.messages['HEARTBEAT'].custom_mode  # this is the flight mode (STAB, LOITER, AUTO, etc)
    # if the vehicle has been armed assume that it is no longer in the home location 

    if (home_location and  (mv.data['BaseMode'] & 128)):
      home_location = False

  except Exception as e:
    print('HEARTBEAT message not recieved yet',e)  


  if ((abs(roll) > math.pi/4) or (abs(pitch)> math.pi/4)):    # if the drone is pitched more than 45 degrees stop stabilize
    roll = 0.0
    pitch = 0.0
 

  if (mv.data['FlightMode'] == 5):  # Loiter
    mode = ST_STABILIZE_VIDEO
  else:
    mode = ST_RAW_VIDEO


  ##############################################
  # State machine for video modes 
  ##############################################

  if mode == ST_RAW_VIDEO:
    # set the final frame to frame
    vu.apply_osd(frame, osd_overlay, mv)

  elif mode == ST_STABILIZE_VIDEO:
    # calc the pixel shift
    dph, dpw = vu.get_pixel_shift(roll, pitch, camera, factor=.7)
    # adjust the vidoe
    frame = vu.image_tranlate(frame, dph, dpw)
    if ((mv.data['Altitude'] > 5) and ((mv.data['Altitude'] < 20))):
      vu.draw_capture_grid(frame, yaw=mv.data['Yaw'], altitude=mv.data['Altitude'])

    # always do this last
    vu.apply_osd(frame, osd_overlay, mv)

  elif mode == ST_FIND_TARGET:
    # calc the pixel shift
    dph, dpw = vu.get_pixel_shift(roll, pitch, camera)
    # stabilize the frame
    frame = vu.image_tranlate(frame, dph, dpw)
    
  # Display the resulting frame
  cv2.imshow('Frame',frame)

  fps.update()

  # Press Q on keyboard to  exit
  key = cv2.waitKey(1) & 0xFF
  if key == ord('q'):
    break
  elif (key == ord('i')):
    # take still and save
    # stop the video stream
    print('Taking still. ')
    vs.capture('image_{}.jpeg'.format(image_number))
    print('Saving to {}'.format('image_{}.jpeg'.format(image_number)))
    image_number += 1



# print the fps   
fps.stop()
print('Frames per second: ',fps.fps())

# When everything done, release the video capture object
vs.stop()

# Closes all the frames
cv2.destroyAllWindows()
