# import required libraries
import cv2
import numpy as np
import time
cap = cv2.VideoCapture(1)
ret, frame = cap.read()
doOnlyOnce = True
first_corners = []
COUNTER_TEST = 0
def addCorners(corners, image):
    global COUNTER_TEST
    newCorners = np.empty((0, 1, 2)) 

    
    red = 50
    rows, cols = 7, 7
    dx = 0
    count = 0
    #Ugly Math
    # Obtengo la media y pongo los circulos
    for i in range(0,len(corners)): #i (0,6) 
        red += 3
        
        x, y = corners[i][0]  # Extrae los valores
       
        
        vector = np.array([[x, y]])
        
        if(not (i % 7 == 0) and i < len(corners) - 1 ):#getHorizontalMean    
            x2, y2 = corners[i+1][0]  # Extrae los valores
            vector_next = np.array([[x2, y2]])
            magnitude = np.linalg.norm((vector_next - vector))
            count += 1
            dx += magnitude
        #image = cv2.circle(image, (int(x), int(y)), 5, (red,red,red), 2)
        #print(str(y) + " : " + str(x))
    dx = dx/count
    
    adyacents = 0
    deltaIndex = 0
    #Agrego los puntos extra??
    for i in range(1,len(corners)): #i (0,6) 
        ##GETTING ONLY de 6TH column
        if(i % 7 == 6):    #crazy ass math 
            xf, yf = corners[i][0]  # Extrae los valores
            xi, yi = corners[i-6][0]  # Extrae los valores
            image = cv2.circle(image, (int(xf), int(yf)), 5, (255,0,0), 5)
            vector = np.array([[xf-xi, yf-yi]])
            #image = cv2.line(image, (int(xi), int(yi)), (int(xf), int(yf)), (0, 255, 0), 2)
            magnitude = np.linalg.norm(vector)
            direction = vector/magnitude #NORMALIZATION
            
            vector = np.array([[xf, yf]])
            newPoint = vector + direction *(dx*0.6)
            xNew, yNew = newPoint[0]
            #image = cv2.circle(image, (int(xNew), int(yNew)), 2, (0,255,0), 2)
            prependElement = np.array([[[xNew, yNew]]])  # Shape (1, 1, 2)
            vector = np.array([[xi, yi]])
            
            newPoint = vector - direction *(dx*0.6)
            xNew, yNew = newPoint[0]
            appendElement = np.array([[[xNew, yNew]]])  # Shape (1, 1, 2)
            slice_arr = corners[i-6:i+1]  # Shape (n, 1, 2)
            slice_arr = np.concatenate((appendElement, slice_arr, prependElement), axis=0) 
            print("len "+str(len(slice_arr)))           
            newCorners = np.concatenate((newCorners,slice_arr), axis=0)
            #newCorners = np.insert(newCorners, (i + deltaIndex) -7 , new_tuple, axis=0)
            #image = cv2.circle(image, (int(xNew), int(yNew)), 2, (0,255,0), 2)
    #Agrego los puntos extra??
    fstSlice = newCorners[0:9].copy()
    
    for i in range(0,len(fstSlice)):
        xf, yf = fstSlice[i][0] 
        xi, yi = newCorners[9*6+i][0] 
        vector = np.array([[xf-xi, yf-yi]]) 

        magnitude = np.linalg.norm(vector)
        
        direction = vector/magnitude #NORMALIZATION
        print(np.linalg.norm(direction))
        vector = np.array([[xf, yf]]) 
        newPoint = vector + direction *(dx*0.6)
        xNew, yNew = newPoint[0]
        fstSlice[i][0] = xNew, yNew 
        #image = cv2.line(image, (int(xi), int(yi)), (int(xf), int(yf)), (0, 255, 0), 2)
    #append pending
    lastSlice = newCorners[(len(newCorners)-9):(len(newCorners))].copy()
    
    for i in range(0,len(lastSlice)):
        xf, yf = lastSlice[i][0] 
        xi, yi = newCorners[len(newCorners)-(9*7-i)][0] 
        image = cv2.line(image, (int(xi), int(yi)), (int(xf), int(yf)), (0, 255, 0), 2)
        
        vector = np.array([[xf-xi, yf-yi]]) 
        
        magnitude = np.linalg.norm(vector)
        
        direction = vector/magnitude #NORMALIZATION
        vector = np.array([[xf, yf]]) 
        newPoint = vector + direction *(dx*0.6)
        xNew, yNew = newPoint[0]
        lastSlice[i][0] = xNew, yNew 
    print(lastSlice)
    newCorners = np.concatenate((fstSlice,newCorners, lastSlice), axis=0)
    COUNTER_TEST = (COUNTER_TEST + 1) %len(newCorners) 
    
    for i in range(0,COUNTER_TEST): #i (0,6) 
        x, y = newCorners[i][0]  # Extrae los valores
        image = cv2.circle(image, (int(x), int(y)), 5, (255,255,red), 2)
    
    #print(len(newCorners))
    #print(newCorners)
    ##PUNTOS FINALES E INICIALES
    x, y = newCorners[0][0]  # Extrae los valores
    image = cv2.circle(image, (int(x), int(y)), 8, (0,0,120), 2)
    x, y = newCorners[len(newCorners)-1][0]  # Extrae los valores
    image = cv2.circle(image, (int(x), int(y)), 8, (0,0,255), 2)
    return image 

while not ret:
    print(ret)
    ret, frame = cap.read()
while True:
    n = 7
    # read input image
    #cap = cv2.VideoCapture(1)
    ret, img = cap.read()
    cv2.waitKey(1)
    # convert the input image to a grayscale
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    
    ## Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (n,n),None)
    #cv2.imshow('Chessboard',img)
    
    # if chessboard corners are detected
    if ret == True:
        if doOnlyOnce:
            first_corners = corners
            print(type(corners))
            print(corners[0])
            addCorners(first_corners, img)
            doOnlyOnce = False
         
        # Draw and display the corners
        #img = cv2.drawChessboardCorners(img, (n,n), corners,ret)
    if not doOnlyOnce:
        img = addCorners(first_corners, img)
        time.sleep(0.1)
        pass
    cv2.imshow('Chessboard',img)