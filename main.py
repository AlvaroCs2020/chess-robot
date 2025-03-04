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
def commandGoToFromAngles(angle1, angle2):
    command = "3;{};{}".format(str(angle1),str(angle2))
    command = "{" + command + "}"
    return command

class Commands():
    C_SET_HOME = "{1;1;1}" 

class States(Enum):
    INIT                = 0 
    HOME                = 1
    MOVE_AWAY           = 2
    MAP_BOARD           = 3
    SET_REAL_CORNERS    = 4
    DETEC_LAST_MOVE     = 5
    EMPTY               = 6
arduino = serial.Serial(port='COM4', baudrate=9600, timeout=.1)
time.sleep(1)#Esperamos que se conecte el arduino

def moveTo(angleFst, angleScnd):
    test = "{1;50;1}"
    arduino.write(bytes(test, 'utf-8'))
    time.sleep(1)
    test = "{2;60;1}"
    arduino.write(bytes(test, 'utf-8'))

state = States.INIT
cap = cv2.VideoCapture(1)
ret, frame = cap.read()
codeDetector = MarkerDetector(frame)
boardProcessor = BoardProcessor()
ikTools = IkUtils(200,150)
window_name = 'main'
home_count = 0
current_position = [0,350]
while not ret:#get video
    print(ret)
    ret, frame = cap.read()
waiting = False
def main(state):
    global home_count
    global waiting
    global goalTime
    ret, frame = cap.read()
    key = cv2.waitKey(1)
    
    currentTime = time.time()
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
        print("[HOME]")
    elif state == States.HOME: 
        if not waiting:
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
                #time.sleep(1)
                arduino.write(bytes(message, 'utf-8'))
                index-=1
            
            home_count+=1
            waiting = True
            #if home_count == 3:
            codeDetector.reset_centers()
            goalTime = time.time() + 2
        elif goalTime <= currentTime:
            command = Commands.C_SET_HOME
            arduino.write(bytes(command, 'utf-8'))
            waiting = False
            state = States.MOVE_AWAY
            print("[MOVE AWAY]")

    elif state == States.MOVE_AWAY:
        if not waiting:
            command = commandGoToFromAngles(50,160)
            arduino.write(bytes(command, 'utf-8'))
            goalTime = currentTime + 5
            waiting = True
        elif goalTime >= currentTime:
            print("[MAP BOARD]")
            state = States.MAP_BOARD
            waiting = False
    elif state == States.MAP_BOARD:
        gray = cv2.cvtColor(ORIGINAL,cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (7,7),None)
        if ret: ##SAVE EMPTY
            boardProcessor.addCorners(corners, ORIGINAL)
            staticSlices, staticCoordinates = boardProcessor.saveSlices(ORIGINAL)
            boardProcessor.setEmptySlices(staticSlices)
            #commandEnterManualMode = "{1;1;1}"
            #arduino.write(bytes(commandEnterManualMode, 'utf-8')) #El robot entra en manual mode
            #HOME
            
            command = "{2;30;0}" #RETURN HOME
            arduino.write(bytes(command, 'utf-8'))
            time.sleep(0.2)
            command = commandGoToFromAngles(90.0,90.0) #RETURN HOME
            arduino.write(bytes(command, 'utf-8'))
            #state = States.SET_REAL_CORNERS
            state = States.DETEC_LAST_MOVE
            print("[DETEC_LAST_MOVE]")
    
    elif state == States.DETEC_LAST_MOVE:
        if key & 0xFF == ord('i'):
            index = boardProcessor.getBiggestError(ORIGINAL)
            print(index)

    elif state == States.SET_REAL_CORNERS:
        
        data = arduino.readline()   #Manual boar calibration
        if key & 0xFF == ord('w'):
            current_position[1] = current_position[1] - 1
            print(current_position)
        if key & 0xFF == ord('s'):
            current_position[1] =current_position[1] + 1
            print(current_position)
        if key & 0xFF == ord('a'):
            current_position[0] =current_position[0] + 1
            print(current_position)
        if key & 0xFF == ord('d'):
            current_position[0] =current_position[0] - 1
            print(current_position)
        
        if key & 0xFF == ord('f'):
            angle1, angle2 = ikTools.compute_angles(current_position[0],current_position[1])
            command = commandGoToFromAngles(angle1, angle2)
            print(command)
            arduino.write(bytes(command, 'utf-8'))
        
        
       
        
        #if data:
        #    print(data.decode('utf-8', errors='ignore').strip())
        #angle1, angle2 = ikTools.compute_angles(150,200)
        #angle1, angle2 = ikTools.compute_angles(0,350)
        #print(str(angle1) + " : " + str(angle2))
    elif state == States.EMPTY:
        
        pass
    #print("estado" + str(state.value))
    codeDetector.set_image(frame)
    if(state.value > 3):
        codeDetector.reset_centers()
        corners, ids, rejected = codeDetector.detection()
        frame = codeDetector.proccesImage(corners, ids, rejected)
        codeDetector.geometryProcessing()
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