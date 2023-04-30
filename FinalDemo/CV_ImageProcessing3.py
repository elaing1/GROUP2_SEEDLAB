'''
Elaine Dang
SEED Group 2
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
from threading import Timer
from threading import Semaphore
import queue
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import smbus2
import serial

'''
Serial Com
'''

ser = serial.Serial('/dev/ttyACM0', 9600)
def ReadfromArduino():
    while (ser.in_waiting > 0):
        try:
            line = ser.readline().decode('utf-8').rstrip()
            return line
        except:
            print("Communication Error")
            

"""
LCD & I2C 
"""
# Modify this if you have a different sized Character LCD
lcd_columns = 16
lcd_rows = 2

# Initialise I2C bus.
#i2c = board.I2C()  # uses board.SCL and board.SDA

# Initialise the LCD class
#lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
#lcd.clear()
#lcd.color = [0, 100, 0]
# for RPI version 1, use “bus = smbus.SMBus(0)”
#bus = smbus2.SMBus(1)
#time.sleep(1)


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
parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
#Create an unbounded shared queue
q = queue.Queue()
screenlock = Semaphore(value=1)
#Aruco id search
global id_order
id_order = 1

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
        #Start Thread
        self.stopped = False
        self.thread2.start()
        
    def show(self):
        while not self.stopped:
            #Display frame
            cv2.imshow('frame', self.frame)
            if cv2.waitKey(1) == ord('q'):
                cv2.destroyAllWindows()
                self.stopped = True
                
    def stop(self):
        self.stopped = True

"""
Image Processing
"""
def ImageProcessing(q, stop, ser):
    start = time.time()
    error = 0
                
    while stop is not True:
        
        frame = q.get()

        id_aruco = id_order

        #Get Optimized Camera Matrix
        h,  w = frame.shape[:2]
        newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))


        #Create parameters for Aruco marker
        para = aruco.DetectorParameters_create()
        para.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

        #DetectAruco & determine ids if any
        corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, arucoDict, parameters=para)

        """
        Finding the angle of the Aruco Marker
        """
        #If Aruco is detected and 6 seconds has passed
        if (id_aruco in ids[0]) and ((time.time() - start) > 1):
            start = time.time()
            
            #Find rotation and translation vector matrix (markerlength = 6.6 cm
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners, 1.968503937, newCameraMatrix, dist)
            z = tvec[0][0][2]

            #Distance Error Correction
            if (z >= 18.5 and z <= 50.5):
                error = np.polyval([(-2 * (10 ** -5)), -0.0011, 0.1108, -1.1113], z)
            
            elif z > 50.5:
                error = np.polyval([0.0202, -2.2323, 61.586], z)
            z = z + error
            z = round(z / 12, 2)

            #Angle Detection -> Left side aruco is 0 (negative - left : positive - right)
            aruco_center = tvec[0][0][0] + markerPoints[0][0][0]
            angle = np.arctan([aruco_center/tvec[0][0][2]])
            angle = round(angle[0] * (180 / np.pi), 1)
            rad = angle * (np.pi / 180)
            
            #Display desired angle on lcd
            screenlock.acquire()
            print("Distance:" + str(z))
            #print(id_aruco)
            #print("Rad: " + str(rad))
            #print("Angle:" + str(angle))
            
            #Add angle threshold
            if (angle < 1 and angle > -1):
                data_angle = "{:.2f}".format(0.0)  # Format the float with 2 decimal places
                data_dist = "{:.2f}".format(z)
                #data_dist = "{:.2f}".format(0.0)
            else:
                #data_angle = "{:.2f}".format(0.0)
                data_angle = "{:.2f}".format(angle)  # Format the float with 2 decimal places
                # was z, now 0
                data_dist = "{:.2f}".format(z)
                #data_dist = "{:.2f}".format(0.0)
            ser.write("<{},{}>".format(data_angle, data_dist).encode())  # Send the formatted floats
            screenlock.release()
            


"""
Main function will display Video Streaming
"""
#Start Threads
video_stream = CamStream(stream_id=0)
video_stream.start()
video_show = CamShow(video_stream.read())
video_show.start()

#image_proc = [Thread(target=ImageProcessing, args=(q, video_stream.stopped,n*3 + 3, )) for n in range(3)]
image_proc = [Timer(n*1, ImageProcessing, args=(q, video_stream.stopped, ser, )) for n in range(2)]

for proc in image_proc:
    proc.start()

#Diplay and Process Frame
aruco_not_flag = 14
while True:
    
    if video_stream.stopped is True:
        break
    
    if video_show.stopped is True:
        break

    frame =  video_stream.read() #Read Frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    #Detect Aruco
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, arucoDict, parameters=parameters)

    #Read Next Aruco Marker

    id_current = ReadfromArduino()
    if id_current is not None:
        id_order = int(id_current)
    '''
    print(id_order)
    if ids is not None:
        q.put(gray)
        aruco_not_flag = 0
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    else:
        if (aruco_not_flag == 7): 
            data_angle = "{:.2f}".format(30)  # Format the float with 2 decimal places
            data_dist = "{:.2f}".format(0)
            ser.write("<{},{}>".format(data_angle, data_dist).encode())  # Send the formatted floats
            print("Distance:" + str(0))
            #ReadfromArduino()
            aruco_not_flag = 0
        aruco_not_flag = aruco_not_flag + 1
    video_show.frame = frame
    '''
    
    print(id_order)
    if ids is not None:
        q.put(gray)
        aruco_not_flag = 0
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    else:
        aruco_not_flag = aruco_not_flag + 1

    if (aruco_not_flag == 15): 
        data_angle = "{:.2f}".format(45)  # Format the float with 2 decimal places
        data_dist = "{:.2f}".format(0)
        ser.write("<{},{}>".format(data_angle, data_dist).encode())  # Send the formatted floats
        ReadfromArduino()
        print("Distance:" + str(0))
        aruco_not_flag = 0
    video_show.frame = frame
       

#Stop video streaming
video_show.stop()
video_stream.stop()


    
