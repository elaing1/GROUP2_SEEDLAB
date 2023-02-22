import cv2 
import time
from picamera import PiCamera
from picamera.array import PiRGBArray
from cv2 import aruco
import numpy as np

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
    
    #Display
    cv2.imshow('frame', gray)
    if cv2.waitKey(1) == ord('q'):
        break
    
cam.release()
cv2.destroyAllWindows()
