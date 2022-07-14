# A function to translate an image by dx, xy pixels

def image_tranlate(img, dx, dy):

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