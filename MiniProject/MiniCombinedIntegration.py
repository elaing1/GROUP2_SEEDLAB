import cv2 
import time
import smbus2
from picamera import PiCamera
from picamera.array import PiRGBArray
from cv2 import aruco
import numpy as np
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import sched

s = sched.scheduler()

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

#Aruco Dictionary
arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_250)

#Create nummpy arrays for Aruco marker data
marker = np.zeros((250, 250, 1), dtype = "uint8")

#Create aruco markers
cv2.aruco.drawMarker(arucoDict, 1, 250, marker, 1)

#Print jpg files of aruco markers
cv2.imwrite('aruco_miniPproject.jpg',marker)

#Create Video Recording
cam = cv2.VideoCapture(0)
rawCapture = PiRGBArray(cam)

Quad = 1
result = "Loading"
prevResult = "None"

#If video cam cannot open exit
if not cam.isOpened():
    print("Cannot open camera")
    exit()
    
setPointDisp = "Not set"
starttime = time.time()
while True:
    #Capture frame by frame
    ret, frame = cam.read() #frame is image array vector

    #If frame is read correctly red it True
    if not ret:
        print("Can't receive frame. Exiting...")
        break;
    #Our operation on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    #Find the center pixel of the image
    center = (gray.shape[1]/2, gray.shape[0]/2)

    #Create parameters for Aruco marker
    parameters = aruco.DetectorParameters_create()

    #DetectAruco & determine ids if any
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, arucoDict, parameters=parameters)

    prevResult = result
    
    if ids is not None:
        
        #Add detection marker on aruco 
        cv2.aruco.drawDetectedMarkers(gray, corners, ids)

        #Find the center pixel of aruco marker
        x_aruco = (corners[0][0][0][0] + corners[0][0][1][0] +corners[0][0][2][0] +corners[0][0][3][0])/4
        y_aruco = (corners[0][0][0][1] + corners[0][0][1][1] +corners[0][0][2][1] +corners[0][0][3][1])/4
        
        #Detect aruco quadrant
        if ((x_aruco > center[0]) and (y_aruco > center[1])):
            Quad = 4
            result = "3pi/2"
        elif ((x_aruco < center[0]) and (y_aruco > center[1])):
            Quad = 3
            result = "pi"
        elif ((x_aruco < center[0]) and (y_aruco < center[1])):
            Quad = 2
            result = "pi/2"
        elif ((x_aruco > center[0]) and (y_aruco < center[1])):
            Quad = 1
            result = "0"  

    else:
        result = "No markers detected"

    #Print only if results changes
    if (result != prevResult and result != "No markers detected"):
        print(result)
        #Writes the quadrant to the UNO, converts to angle there
        bus.write_byte_data(0x04, 0, Quad)
    
        #Display desired angle on lcd
        lcd.color = [0, 100, 0]
        lcd.clear()
        setPointDisp = "Setpoint: " + result
        lcd.message = setPointDisp
    
    

    #Display
    cv2.imshow('frame', gray)
    if cv2.waitKey(1) == ord('q'):
        break
    
    
    
    #Reads current position from UNO
    readPosition = bus.read_i2c_block_data(0x04, 0, 2)
        
    #bitshifts back to full number
    fullPosition = (readPosition[0] << 8) | readPosition[1]
    #print(fullPosition)
        
    #time.sleep(1)
        
    #converts to actual angle
    actualPosition = fullPosition/3210 * 2*3.14;
            
    #Displays current position along with desired setpoint on two lines
      
    fullMessage = setPointDisp + "\nActual: " + str(actualPosition)
    lcd.message = fullMessage
        
    
cam.release()
cv2.destroyAllWindows()
