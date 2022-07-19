from math import tan, degrees, radians

import cv2

# A function to calculate pixel shift based on roll/pitch and camera properties

# sensor and camera properties
sensor_IMX219 = {
    'name': 'Sony IMX219 CMOS color 8 Megapixel',
    'size_h_w':(2.76, 3.674),
    'pixels_h_w':(2464, 3296),
    'pixel_size':(0.00112,0.00112),
    'sensor_mode':{
        1:{
            'resolution':'1920x1080',
            'res_h':1080,
            'res_w':1920,
            'aspect_ratio':'16:9',
            'frame_rates':'1 < fps <= 30',
            'max_frame_rate':30,
            'binning':1
        },
        2:{
            'resolution':'3280x2464',
            'res_h':2464,
            'res_w':3280,
            'aspect_ratio':'4:3',
            'frame_rates':'1 < fps <= 15',
            'max_frame_rate':15,
            'binning':1
        },
        3:{
            'resolution':'3280x2464',
            'res_h':2464,
            'res_w':3280,
            'aspect_ratio':'4:3',
            'frame_rates':'1 <= fps <= 15',
            'max_frame_rate':15,
            'binning':1
        },
        4:{
            'resolution':'1640x1232',
            'res_h':1232,
            'res_w':1640,
            'aspect_ratio':'4:3',
            'frame_rates':'1 < fps <= 40',
            'max_frame_rate':40,
            'binning':2
        },
        5:{
            'resolution':'1640x922',
            'res_h':922,
            'res_w':1640,
            'aspect_ratio':'16:9',
            'frame_rates':'1 < fps <= 40',
            'max_frame_rate':40,
            'binning':2
        },
        6:{
            'resolution':'1280x720',
            'res_h':720,
            'res_w':1280,
            'aspect_ratio':'16:9',
            'frame_rates':'40 < fps <= 90',
            'max_frame_rate':90,
            'binning':2
        },
        7:{
            'resolution':'640x480',
            'res_h':480,
            'res_w':640,
            'aspect_ratio':'4:3',
            'frame_rates':'40 < fps <= 90',
            'max_frame_rate':90,
            'binning':2
        }
    }
}

# image area: 3673.6 μm x 2738.4 μm
sensor_OV5647 = {
    'name': 'Omnivision OV5647 CMOS color 5 Megapixel',
    'size_h_w':(2.738, 3.674),
    'pixels_h_w':(1944, 2592),
    'pixel_size':(0.0014,0.0014),
    'sensor_mode':{
        1:{
            'resolution':'1920x1080',
            'res_h':1080,
            'res_w':1920,
            'aspect_ratio':'16:9',
            'frame_rates':'1 < fps <= 30',
            'max_frame_rate':30,
            'binning':1
        },
        2:{
            'resolution':'2592x1944',
            'res_h':1944,
            'res_w':2592,
            'aspect_ratio':'4:3',
            'frame_rates':'1 < fps <= 15',
            'max_frame_rate':15,
            'binning':1
        },
        3:{
            'resolution':'2592x1944',
            'res_h':1944,
            'res_w':2592,
            'aspect_ratio':'4:3',
            'frame_rates':'1/6 <= fps <= 1',
            'max_frame_rate':1,
            'binning':1
        },
        4:{
            'resolution':'1296x972',
            'res_h':972,
            'res_w':1296,
            'aspect_ratio':'4:3',
            'frame_rates':'1 < fps <= 42',
            'max_frame_rate':42,
            'binning':2
        },
        5:{
            'resolution':'1296x730',
            'res_h':730,
            'res_w':1296,
            'aspect_ratio':'16:9',
            'frame_rates':'1 < fps <= 49',
            'max_frame_rate':49,
            'binning':2
        },
        6:{
            'resolution':'640x480',
            'res_h':480,
            'res_w':640,
            'aspect_ratio':'16:9',
            'frame_rates':'1 < fps <= 49',
            'max_frame_rate':49,
            'binning':2
        },
        7:{
            'resolution':'1296x730',
            'res_h':730,
            'res_w':1296,
            'aspect_ratio':'16:9',
            'frame_rates':'1 < fps <= 49',
            'max_frame_rate':49,
            'binning':2
        }
    
    }
}


def configure_camera(sensor, lens_f=None, image_mode=None, frame_rate=None):
    cam = {}
    cam['sensor'] = sensor
    cam['lens_f'] = lens_f
    cam['image_mode'] = image_mode
    cam['frame_rate'] =  frame_rate

    return cam



def get_pixel_shift(roll, pitch, camera, factor=1.0):
    dpw = -1 * factor * (camera['sensor']['pixels_h_w'][1]     # sensor px
           * camera['lens_f']                    # focal length
           / camera['sensor']['sensor_mode'][camera['image_mode']]['binning'] # binning factor
           / camera['sensor']['size_h_w'][1]
           * tan(roll))

    dph = -1 * factor * (camera['sensor']['pixels_h_w'][0]     #sensor px
                * camera['lens_f']                    # focal length
                / camera['sensor']['sensor_mode'][camera['image_mode']]['binning'] # binning factor
                / camera['sensor']['size_h_w'][0]
                * tan(pitch))

    return dph, dpw



# A function to translate an image by dx, xy pixels very quickly
def image_tranlate(img, dy, dx):

    import cv2
    import numpy as np
    # set up the transformation matrix
    M = np.float32([
	[1, 0, int(dx)],
	[0, 1, int(dy)]])

    return cv2.warpAffine(img, M, (img.shape[1], img.shape[0]))


import datetime
# A class to meausure the frames per second 
class FPS:
    
    def __init__(self):
		# store the start time, end time, and total number of frames
		# that were examined between the start and end intervals
        self._start = None
        self._end = None
        self._numFrames = 0
    def start(self):
		# start the timer
        self._start = datetime.datetime.now()
        return self
    def stop(self):
		# stop the timer
        self._end = datetime.datetime.now()
    def update(self):
		# increment the total number of frames examined during the
		# start and end intervals
        self._numFrames += 1
    def elapsed(self):
        # return the total number of seconds between the start and
        # end interval
        return (self._end - self._start).total_seconds()
    def fps(self):
        # compute the (approximate) frames per second
        return self._numFrames / self.elapsed()


font = cv2.FONT_HERSHEY_SIMPLEX

def draw_altitude(frame, mv):
    cv2.putText(frame, 'Alt:{:2.1f}'.format(mv.data['Altitude']*3.28), (550, 25), font, 1, (0, 0, 255), 2, cv2.LINE_AA)

def draw_lat_lon(frame, mv):
    cv2.putText(frame, 'Lat:{:.7f}'.format(mv.data['Lat']), (5, 25), font, 1, (255,0, 0), 2, cv2.LINE_AA)
    cv2.putText(frame, 'Lon:{:.7f}'.format(mv.data['Lat']), (280, 25), font, 1, (255,0, 0), 2, cv2.LINE_AA)

def draw_battery(frame, mv):
    cv2.putText(frame, 'B:{:3.1f}V  {:3d}%'.format(mv.data['BattV'], int(mv.data['BattPercent'])), (5, 570), font, 1, (255,255, 255), 2, cv2.LINE_AA)

def draw_cross_hairs(frame):

    cv2.line(frame, (360,248), (360,328), (0, 0, 255), 3) 
    cv2.line(frame, (320,288), (400,288), (0, 0, 255), 3) 

def draw_flight_mode(frame, mv):
    cv2.putText(frame, 'FM:{}'.format(mv.data['FlightMode']), (550, 570), font, 1, (0,255, 0), 2, cv2.LINE_AA)


def apply_osd(frame, osd, mv):
    # possibly move this to functions:

    if (osd):
        if (osd == 'Screen1'):
            # print altitude
            draw_altitude(frame, mv)
            draw_lat_lon(frame, mv)
            draw_battery(frame, mv)
            draw_flight_mode(frame, mv)
            draw_cross_hairs(frame)
        elif (osd == 'Screen2'):
            cv2.putText(frame, 'A:{:2.1f}'.format(mv.data['Altitude']), (650, 25), font, 1, (0, 0, 255), 2, cv2.LINE_AA)


    else:
        return frame

class Mavlink:

    data = {
        'Altitude':13,
        'Lat':33.521117, 
        'Lon':-84.581361, 
        'BattV':14.5,
        'BattPercent':45,
        'FlightMode':'STAB',
        'DisplayMode':'st'
    }

from pymavlink import mavutil

def request_message_interval(master, message_id: int, frequency_hz: float):
    """
    Request MAVLink message in a desired frequency,
    documentation for SET_MESSAGE_INTERVAL:
        https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

    Args:
        message_id (int): MAVLink message ID
        frequency_hz (float): Desired frequency in Hz
    """
    
    if (frequency_hz > 0):
        message_freq_us = 1e6 / frequency_hz
    elif (frequency_hz == 0):
        message_freq_us = 0   # leave default
    else:
        message_freq_us = -1   # disable

    print('Requesting message frequency: {}  -  {}hz  {}us'.format(message_id, frequency_hz, message_freq_us))

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # The MAVLink message ID
        message_freq_us, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
        0, 0, 0, 0, # Unused parameters
        0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    )

