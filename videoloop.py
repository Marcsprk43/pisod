# Main video loop

import cv2
import numpy as np
import videoutils as vu

# this is simple class to calculate frames per second
fps = vu.FPS()

sensor = vu.sensor_IMX219 # select the PiCamera V2.1 sensor

camera = vu.configure_camera(sensor, lens_f=2.1, image_mode=1, frame_rate=1)



# Create a VideoCapture object and read from input file
# If the input is the camera, pass 0 instead of the video file name
cap = cv2.VideoCapture(0)

cap.set(3, 1250)  # Set horizontal resolution
cap.set(4, 720)  # Set vertical resolution

# Check if camera opened successfully
if (cap.isOpened()== False): 
  print("Error opening video stream or file")

fps.start()


while(1):
    
  # Capture frame-by-frame
  ret, frame = cap.read()
  if ret == True:

    shift_frame = vu.image_tranlate(frame, 200,200)
    
    # Display the resulting frame
    cv2.imshow('Frame',shift_frame)

    fps.update()

    # Press Q on keyboard to  exit
    if cv2.waitKey(25) & 0xFF == ord('q'):
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
