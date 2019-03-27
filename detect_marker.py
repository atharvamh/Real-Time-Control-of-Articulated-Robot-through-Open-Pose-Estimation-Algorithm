import numpy as np
import cv2.aruco as aruco
import cv2
import math

markerLength = 100

cap = cv2.VideoCapture(0)
while(True):
    ret,frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)    
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    parameters = aruco.DetectorParameters_create()

    #getting aruco_image corners and aruco_id
    corners, aruco_id, _ = aruco.detectMarkers(gray,aruco_dict,parameters = parameters)    
    #drawing detected markers
    image1 = aruco.drawDetectedMarkers(frame.copy(),corners,aruco_id)
    cv2.imshow('Aruco_image',image1)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
