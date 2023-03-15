'''
Description:
Determine the angle and distance of the aruco marker
'''

#Import Libraries
import cv2
import time
import numpy as np
from picamera import PiCamera
from cv2 import aruco
from threading import Thread
from threading import Semaphore
import queue
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import smbus2

"""
LCD & I2C 
"""
# Modify this if you have a different sized Character LCD
lcd_columns = 16
lcd_rows = 2

# Initialise I2C bus.
i2c = board.I2C()  # uses board.SCL and board.SDA

# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.clear()
lcd.color = [0, 100, 0]
# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus2.SMBus(1)
time.sleep(1)


#this is the address we setup in the Arduino Program
address = 0x04

"""
Creating Aruco
"""
#Aruco Dictionary
arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_250)
#Create nummpy arrays for Aruco marker data
marker = np.zeros((250, 250, 1), dtype = "uint8")
#Create aruco marker
cv2.aruco.drawMarker(arucoDict, 1, 250, marker, 1)
#Write aruco marker to jpg file
cv2.imwrite('aruco_miniPproject.jpg',marker)
#Create parameters for Aruco marker
parameters = aruco.DetectorParameters_create()
#Create an unbounded shared queue
q = queue.Queue()
screenlock = Semaphore(value=1)

"""
Open calibration (camera matrix and distortion)
"""
cali = np.load('calibration.npz')
cameraMatrix = cali['camMatrix']
dist = cali['distortion']


"""
Read Video Capture Streaming
"""

class CamStream:
    def __init__(self, stream_id=0):
        #Initilize camera capture
        self.cam = cv2.VideoCapture(0)
        
        #Check if Camera can be accessed
        if not self.cam.isOpened():
            print("[Exiting]: Error opening camera")
            exit()

        #Read in first frame
        self.ret, self.frame = self.cam.read()

        if self.ret is False:
            print("[Exiting]: No frames to read")
            exit()

        #Initialized stop to True
        self.stopped = True

        #Thread Instantiation
        self.thread1 = Thread(target=self.update, args=())
        

        
    #method to start thread       
    def start(self):
        self.stopped = False
        self.thread1.start()
        
    #method to read next available frame    
    def update(self):
        while True:
            if self.stopped is True :
                break
            self.ret, self.frame = self.cam.read()
            if self.ret is False:
                print("[Exiting]: No frames to read")
                self.stopped = True
                break
        self.cam.release()

        
    #method to return latest read frame    
    def read(self):
        return self.frame

    #method to stop reading frames
    def stop(self):
        self.stopped =True

"""
Show Video Capture Streaming
"""
class CamShow:
    def __init__(self, frame=None):
        self.frame = frame
        #Initialized stop to True
        self.stopped = True
        #Thread initilization
        self.thread2 = Thread(target=self.show, args=())
        
    def start(self):
        self.stopped = False
        self.thread2.start()
        
    def show(self):
        while not self.stopped:
            #Display
            cv2.imshow('frame', self.frame)
            if cv2.waitKey(1) == ord('q'):
                cv2.destroyAllWindows()
                self.stopped = True
                
    def stop(self):
        self.stopped = True

"""
LCD Display
"""




"""
Image Processing
"""
def ImageProcessing(q, stop):
    start = time.time()
    while stop is not True:
        frame = q.get()
        """
        Fixing Distortion with Camera Matrix and distortion coefficients
        """

        #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h,  w = frame.shape[:2]
        newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))

        # Undistort
        dst = cv2.undistort(frame, cameraMatrix, dist, None, newCameraMatrix)

        # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]

        #Create parameters for Aruco marker
        parameters = aruco.DetectorParameters_create()

        #DetectAruco & determine ids if any
        corners, ids, rejectedImgPoints = aruco.detectMarkers(dst, arucoDict, parameters=parameters)

        """
        Finding the angle of the Aruco Marker
        """
        
        if ids is not None:

            #Find the center pixel of aruco marker
            x_aruco = (corners[0][0][0][0] + corners[0][0][1][0] +corners[0][0][2][0] +corners[0][0][3][0])/4
            y_aruco = (corners[0][0][0][1] + corners[0][0][1][1] +corners[0][0][2][1] +corners[0][0][3][1])/4

            #Find Angle
            #Convert 2D point to homogenous point (x,y,1)
            image_pt = np.array([x_aruco, y_aruco, 1])

            center_pt = np.array([dst.shape[1]/2, y_aruco, 1])
            
            #Multiply that by the inverse of the camera intrinisic matrix
            world_pt = np.linalg.inv(newCameraMatrix).dot(image_pt)
            center_pt = np.linalg.inv(newCameraMatrix).dot(center_pt)

            if (world_pt[0] == center_pt[0]):
                angle = 0.00
            else:
                #find angle through dot product formula
                angle = np.arccos(world_pt.dot(center_pt) / (np.linalg.norm(world_pt) * np.linalg.norm(center_pt)))
                angle = round(angle * (180 / np.pi), 0)
                if (world_pt[0] > center_pt[0]):
                    angle = -1 * angle
            
            screenlock.acquire()
            print(angle)
            print(round((time.time() - start)) % 10)
            if (((time.time() - start) % 10) == 0):
                print(time.time() - start)
                #Display desired angle on lcd
                setPointDispAruco = "Aruco Detected\n"
                setPointDisp = "Angle: " + str(angle)
                lcd.message = setPointDispAruco + setPointDisp
            screenlock.release()
            #Declare Origin


   
            


"""
Main function will display Video Streaming
"""
#Start Threads
video_stream = CamStream(stream_id=0)
video_stream.start()
video_show = CamShow(video_stream.read())
video_show.start()
image_proc = [Thread(target=ImageProcessing, args=(q, video_stream.stopped, )) for _ in range(3)]
for proc in image_proc:
    proc.start()
#display_angle = Thread(target=lcdDisplay, args=())

while True:
    
    if video_stream.stopped is True:
        break
    
    if video_show.stopped is True:
        break

    
    frame =  video_stream.read()
    q.put(frame)

    #DetectAruco & determine ids if any
    corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, arucoDict, parameters=parameters)
        
    #If Aruco id is detected, draw marker and queue frame    
    if ids is not None:
        #Add detection marker on aruco

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #q.put(gray)
        
    video_show.frame = frame

    
    

    
#Stop video streaming
video_show.stop()
video_stream.stop()


    
