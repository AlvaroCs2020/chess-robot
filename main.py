from MarkerDetector import MarkerDetector # type: ignore
from BoardProcessor import BoardProcessor
from IkUtils import IkUtils # type: ignore
import argparse
import imutils # type: ignore
import cv2 # type: ignore
import sys
import numpy as np # type: ignore
import math
import serial # type: ignore
import time 
from enum import Enum

class States(Enum):
    INIT                = 0 
    HOME                = 1
    MOVE_AWAY           = 2
    MAP_BOARD           = 3
    SET_REAL_CORNERS    = 4
    
state = States.INIT

cap = cv2.VideoCapture(1)
ret, frame = cap.read()
codeDetector = MarkerDetector(frame)
boardProcessor = BoardProcessor()
ikTools = IkUtils(150,200)
window_name = 'main'
home_count = 0
while not ret:#get video
    print(ret)
    ret, frame = cap.read()

arduino = serial.Serial(port='COM4', baudrate=9600, timeout=.1)
time.sleep(1)#Esperamos que se conecte el arduino

def main(state):
    global home_count
    ret, frame = cap.read()
    key = cv2.waitKey(1)
    ORIGINAL = frame.copy()
    
    if state == States.INIT:
      print("[INIT]")
      
      if(not codeDetector.is_ready()):
        codeDetector.set_image(frame)
        corners, ids, rejected = codeDetector.detection()
        frame2 = codeDetector.proccesImage(corners, ids, rejected)
        codeDetector.set_image(frame2)
        
        #codeDetector.reset_centers()
      else:
        state = States.HOME

    elif state == States.HOME:

        print("[HOME]")
        codeDetector.geometryProcessing()
        #state = States.INIT
        index = 2
        #Enviamos mensaje al arduino con el motor y los angulos que necesitamos
        for i in codeDetector.get_angles(): 
            sense = 1
            
            if(index == 1):
                if(0 < -i):
                    sense = 0
            else:
                if(0 < i):
                    sense = 0

            message = "{};{};{}".format(index, abs(i), sense)
            message = "{" + message + "}"
            print("Message sent to arduino: "+ message)
            print("Angle for this: "+ str(i))
            time.sleep(1)
            arduino.write(bytes(message, 'utf-8'))
            
            index-=1
        home_count+=1
        #if home_count == 3:
        codeDetector.reset_centers()
        state = States.MOVE_AWAY
    elif state == States.MOVE_AWAY:
        print("[MOVE AWAY]")
        test = "{1;50;1}"
        arduino.write(bytes(test, 'utf-8'))
        time.sleep(1)
        test = "{2;60;1}"
        arduino.write(bytes(test, 'utf-8'))
        time.sleep(1)
        state = States.MAP_BOARD
    elif state == States.MAP_BOARD:
        print("[MAP BOARD]")
        gray = cv2.cvtColor(ORIGINAL,cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (7,7),None)
        if ret: ##SAVE EMPTY
            boardProcessor.addCorners(corners, ORIGINAL)
            staticSlices, staticCoordinates = boardProcessor.saveSlices(ORIGINAL)
            boardProcessor.setEmptySlices(staticSlices)
            commandEnterManualMode = "{1;1;1}"
            arduino.write(bytes(commandEnterManualMode, 'utf-8'))
            state = States.SET_REAL_CORNERS
    elif state == States.SET_REAL_CORNERS:
        print("[SET_REAL_CORNERS]")
        angle1, angle2 = ikTools.compute_angles(150,200)
        angle1, angle2 = ikTools.compute_angles(0,350)
        print(str(angle1) + " : " + str(angle2))

    codeDetector.set_image(frame)
    frame = codeDetector.get_image()
    frame = boardProcessor.GetTechnicView(frame)
    cv2.imshow(window_name, frame)    
    
    if key & 0xFF == ord('r'):
        print("Reset centers of Markers")
        test = "{2;38.617;0}"
        arduino.write(bytes(test, 'utf-8'))
        #codeDetector.reset_centers()
        #state = States.INIT

    return state


while True:
    state = main(state)