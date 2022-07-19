# Main video loop
import cv2
import numpy as np
from imutils.video.pivideostream import PiVideoStream
import videoutils as vu
from pymavlink import mavutil
import time
from picamera import PiCamera
import math

####################################################################
# Video system states
####################################################################
ST_RAW_VIDEO = 0
ST_STABILIZE_VIDEO = 1
ST_FIND_TARGET = 2

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
framerate = 20

image_number = 0 # for still images filename

def SetupCamera(resolution, framerate):
    # initialize the video stream and allow the cammera sensor to warmup
    vs = PiVideoStream(resolution=resolution, framerate=framerate)
    return vs




sensor = vu.sensor_IMX219 # select the PiCamera V2.1 sensor
camera = vu.configure_camera(sensor, lens_f=2.1, image_mode=5, frame_rate=15)

# initialize the video stream and allow the cammera sensor to warmup
vs = SetupCamera(resolution=resolution, framerate=framerate)
vs.start()

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
  #roll, pitch = get_roll_pitch()

  try: 
    
    m = the_connection.recv_msg()
    
    msg_tries = 0
    while not (m is None) and msg_tries < 2:  # read in all the accumulated mavlink messages
      m = the_connection.recv_msg()
      msg_tries += 1

    #the_connection.recv_match(blocking=False)
    mv.data['Lat'] = the_connection.messages['AHRS2'].lat  # Note, you can access message fields as attributes!
    mv.data['Lon'] = the_connection.messages['AHRS2'].lng  # Note, you can access message fields as attributes!
    mv.data['Altitude'] = the_connection.messages['AHRS2'].altitude  # Note, you can access message fields as attributes!
    mv.data['BattV'] = the_connection.messages['SYS_STATUS'].voltage_battery  # Note, you can access message fields as attributes!
    mv.data['BattPercent'] = the_connection.messages['SYS_STATUS'].battery_remaining
    mv.data['BaseMode'] = the_connection.messages['HEARTBEAT'].base_mode  # the base_mode describes the armed, disarmed, etc. status
    mv.data['FlightMode'] = the_connection.messages['HEARTBEAT'].custom_mode  # this is the flight mode (STAB, LOITER, AUTO, etc)

    roll = the_connection.messages['AHRS2'].roll
    pitch = the_connection.messages['AHRS2'].pitch  
      
  except Exception as e:
      print(e, 'not received yet')
      roll = 0.0
      pitch = 0.0

  if ((abs(roll) > math.pi/4) or (abs(pitch)> math.pi/4)):    # if the drone is pitched more than 45 degrees stop stabilize
    roll = 0.0
    pitch = 0.0
 

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
  key = cv2.waitKey(1) & 0xFF
  if key == ord('q'):
    break
  elif (key == ord('i')):
    # take still and save
    # stop the video stream
    print('Stopping video')
    vs.capture('~/image_{}.jpeg')



    

# print the fps   
fps.stop()
print('Frames per second: ',fps.fps())

# When everything done, release the video capture object
vs.stop()

# Closes all the frames
cv2.destroyAllWindows()
