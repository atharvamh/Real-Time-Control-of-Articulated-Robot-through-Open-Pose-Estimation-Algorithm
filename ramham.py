#---------------IMPORTING LIBRARIES-------------------------------------------------------------------------------------------------

import cv2
import cv2.aruco as aruco
import time
import numpy as np
import serial
import math


#------------------------------DECLARATION OF VARIABLES-----------------------------------------------------------------------------

markerLength = 100                                                          #declaring the number of aruco_markers
font = cv2.FONT_HERSHEY_SIMPLEX                                             #specify the font used on writing images/videos

#Declare the serial object for serial communication with arduino
#check the serial port (com7 - this might change). The baud rate(9600) can also be varied.

ArduinoSerial = serial.Serial('com7',9600)                                  
time.sleep(2)
#------------------------------------------------------------------------------------------------------------------------------------


#specify path for model weight file
protoFile = "pose_deploy_linevec_faster_4_stages.prototxt"
weightsFile = "pose_iter_160000.caffemodel"                                 #using the pre-trained model for pose detection

#------------------------------------------------------------------------------------------------------------------------------------

#Key points on the body detected adjacent to each other are grouped into pairs

POSE_PAIRS = [[0,1], [1,2], [2,3], [3,4], [1,5], [5,6], [6,7], [1,14], [14,8], [8,9], [9,10], [14,11], [11,12], [12,13] ]
 
nPoints = 15                                                                #specifies the number of key points detected on the body

#------------------------------------------------------------------------------------------------------------------------------------

# specify the input video dimensions

inWidth = 120
inHeight = 120
threshold = 0.1

#---------------------------VIDEO CAPTURE MODULE--------------------------------------------------------------------------------------

cap = cv2.VideoCapture(1)
hasFrame,frame = cap.read()

#--------------------------THE DNN MODULE---------------------------------------------------------------------------------------------

#Read the Caffe network into memory

net = cv2.dnn.readNetFromCaffe(protoFile,weightsFile)

#-------------------------------------------------------------------------------------------------------------------------------------

while(True):
    hasFrame, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)      #grayscale conversion of existing frame

    #-----------------ARUCO MARKER DETECTION---------------------------------------------
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    parameters = aruco.DetectorParameters_create()
    corners, aruco_id, _ = aruco.detectMarkers(gray,aruco_dict,parameters = parameters)  
    image1 = aruco.drawDetectedMarkers(frame.copy(),corners,aruco_id)
    #------------------------------------------------------------------------------------
    
    frame = cv2.flip(frame,1)                           #flipping the frame
    frameCopy = np.copy(frame)                          #creating a copy of the frame
    subtracted = frame - frame
    roi = frame - frame
    

    if not hasFrame:
        cv2.waitKey()
        break

    #------------------------------------------------------------------------------------

    frameWidth = frame.shape[1]
    frameHeight = frame.shape[0]

    #Feeding the frame from the video to the network
    inpBlob = cv2.dnn.blobFromImage(frame,1.0/255,(inWidth,inHeight),
                                     (0,0,0),swapRB = False,crop = False)

    
    #set the object as input to the network
    net.setInput(inpBlob)
    output = net.forward()

    # The output will be a 4D matrix
    # first dimension is image ID if more than one image passed
    # second dimension is the index of keypoint
    # third dimension is the height(second element of the array)
    # fourth dimension is the width(third element of the array)
    
    H = output.shape[2]
    W = output.shape[3]

    
    #Empty list to store detected keypoints
    points = []

    for i in range(nPoints):

        #Confidence Map of corresponding body's part
        probMap = output[0,i,:,:]
        
        # Find global maxima of the probMap ( probability Map of a point )
        minVal , prob , minLoc , point = cv2.minMaxLoc(probMap)

        x = (frameWidth * point[0])/W
        y = (frameHeight * point[1])/H

        if prob > threshold :
            cv2.circle(frameCopy, (int(x),int(y)),8,(0,255,255),thickness = -1,lineType = cv2.FILLED)

            points.append((int(x),int(y)))
        else:
            points.append(None)

    for pair in POSE_PAIRS:
        partA = pair[0]
        partB = pair[1]

        #print(partA)
        #print(partB)
        #print("\n")

        if points[partA] and points[partB]:
            cv2.line(frame,points[partA],points[partB],(0,255,255),3,lineType = cv2.LINE_AA)
            cv2.circle(frame, points[partA], 8, (0, 0, 255), thickness=-1, lineType=cv2.FILLED)
            cv2.putText(frame,str(partA),points[partA], font, 1,(0,255,0),2,cv2.LINE_AA)
            cv2.circle(frame, points[partB], 8, (0, 0, 255), thickness=-1, lineType=cv2.FILLED)
            cv2.putText(frame,str(partB),points[partB], font, 1,(0,255,0),2,cv2.LINE_AA)
            
            cv2.line(subtracted,points[partA],points[partB],(0,255,255),3,lineType = cv2.LINE_AA)
            cv2.circle(subtracted, points[partA], 8, (0, 0, 255), thickness=-1, lineType=cv2.FILLED)
            cv2.putText(subtracted,str(partA),points[partA], font, 1,(0,255,0),2,cv2.LINE_AA)
            cv2.circle(subtracted, points[partB], 8, (0, 0, 255), thickness=-1, lineType=cv2.FILLED)
            cv2.putText(subtracted,str(partB),points[partB], font, 1,(0,255,0),2,cv2.LINE_AA)

        
        if points[partA] and points[partB] and (partA==2 and partB==3) or (partA==3 and partB==4):
            cv2.line(roi,points[partA],points[partB],(0,255,255),3,lineType = cv2.LINE_AA)
            cv2.circle(roi, points[partA], 8, (0, 0, 255), thickness=-1, lineType=cv2.FILLED)
            cv2.putText(roi,str(partA),points[partA], font, 1,(0,255,0),2,cv2.LINE_AA)
            cv2.circle(roi, points[partB], 8, (0, 0, 255), thickness=-1, lineType=cv2.FILLED)
            cv2.putText(roi,str(partB),points[partB], font, 1,(0,255,0),2,cv2.LINE_AA)
            #print(partA,partB)

    #---------------EXTRACTION OF ARM FROM THE COMPLETE BODY POSE--------------------------------------
    shoulder = points[2]
    elbow = points[3]
    wrist = points[4]
    if shoulder != None and elbow != None and wrist != None:
        if shoulder[0]-elbow[0] != 0 and elbow[0]-wrist[0] != 0:
            shoulder_angle = 180/np.pi * np.arctan((shoulder[1]-elbow[1])/(shoulder[0]-elbow[0]))
            elbow_angle = 180/np.pi * np.arctan((elbow[1]-wrist[1])/(elbow[0]-wrist[0]))
            cv2.putText(roi,str(shoulder_angle),points[2], font, 1,(0,255,0),2,cv2.LINE_AA)
            cv2.putText(roi,str(elbow_angle),points[3], font, 1,(0,255,0),2,cv2.LINE_AA)
            
            
            shoulder_angle = abs(int(shoulder_angle))
            elbow_angle = abs(int(elbow_angle))

            #----------RELATION FOR CALIBRATION ONTO THE BOT ARM (TRIAL AND ERROR)----------------------
            
            shoulder_angle = int(1.6*shoulder_angle + 50)
            elbow_angle = int(0.7*elbow_angle + 120)

            #-------------------------------------------------------------------------------------------

            #---------------------CONSTRAINTS ON THE BOT ARM MOVEMENT-----------------------------------
            
            if shoulder_angle >= 40 and shoulder_angle <= 60:
                a1 = shoulder_angle
                b1 = 120
                
            elif shoulder_angle > 60 and shoulder_angle <= 80:
                if elbow_angle >= 160 and elbow_angle <= 180:
                    a1 = shoulder_angle
                    b1 = elbow_angle
                else:
                    a1 = shoulder_angle
                    if (elbow_angle < 160):
                        b1 = 160
                    else:
                        b1 = 180                        
                
            elif shoulder_angle > 80 and shoulder_angle <= 100:
                if elbow_angle >= 140 and elbow_angle <= 180:
                    a1 = shoulder_angle
                    b1 = elbow_angle
                else:
                    a1 = shoulder_angle
                    if (elbow_angle < 140):
                        b1 = 140
                    else:
                        b1 = 180  
            
            elif shoulder_angle > 100 and shoulder_angle <= 150:
                if elbow_angle >= 120 and elbow_angle<= 160:
                    a1 = shoulder_angle
                    b1 = elbow_angle
                else:
                    a1 = shoulder_angle
                    if (elbow_angle < 120):
                        b1 = 120
                    else:
                        b1 = 160  
            
            elif shoulder_angle>150 and shoulder_angle<=170:
                if elbow_angle>=120 and elbow_angle<=160:
                    a1 = shoulder_angle
                    b1 = elbow_angle
                else:
                    a1 = shoulder_angle
                    if (elbow_angle < 120):
                        b1 = 120
                    else:
                        b1 = 160  
            
            elif shoulder_angle>170 and shoulder_angle<=180:
                if elbow_angle>=120 and elbow_angle<=140:
                    a1=shoulder_angle
                    b1=elbow_angle
                else:
                    a1 = shoulder_angle
                    if (elbow_angle < 120):
                        b1 = 120
                    else:
                        b1 = 140  
            else:
                a1 = 40
                b1 = 180

            
            shoulder_angle = a1
            elbow_angle = b1
            
            print(shoulder_angle,elbow_angle)

            #---------------------------------------------------------------------------------

            #sending the shoulder and elbow angles as as string to arduino - 8 character string
            SA_str = str(shoulder_angle)
            EA_str = str(elbow_angle)

            if(abs(shoulder_angle) < 10):
                SA_str = '00'+SA_str
        
            elif(abs(shoulder_angle) < 100):
                SA_str = '0'+SA_str    

            if(abs(elbow_angle) < 10):
                EA_str = '00'+EA_str

            elif(abs(elbow_angle) < 100):
                EA_str = '0'+EA_str

            if(len(corners)):
                Ar_str = str(1)
            else:
                Ar_str = str(0)

            # The data string has 8 characters - first for marker detected or not(1 or 0)
            # The next three characters - 3 digit shoulder angle
            # The next three characters - 3 digit elbow angle
            # The last character as the endline to denote the transmission of one packet of data
            
            data = Ar_str + SA_str + EA_str + '\n'
            ArduinoSerial.write(data.encode())                          #Writing the data string to arduino
            print(data)


    #--------------------IMAGE SHOWING UNIT ( OUTPUT UNIT ) ------------------------------------------------------------------        
            
    cv2.imshow('Aruco_image',image1)
    cv2.imshow('Skeleton',frame)
    cv2.imshow('StickFigure',subtracted)
    cv2.imshow('arm',roi)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):                   #when q pressed by the user when program is running force exit the program
        break

cap.release()
cv2.destroyAllWindows()
