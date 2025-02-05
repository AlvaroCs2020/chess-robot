# import required libraries
import cv2
import numpy as np
cap = cv2.VideoCapture(1)
ret, frame = cap.read()
doOnlyOnce = True
first_corners = []
def addCorners(corners, image):
    ##PUNTOS FINALES E INICIALES
    x, y = corners[0][0]  # Extrae los valores
    image = cv2.circle(image, (int(x), int(y)), 8, (0,0,120), 2)
    x, y = corners[len(corners)-1][0]  # Extrae los valores
    image = cv2.circle(image, (int(x), int(y)), 8, (0,0,255), 2)
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
        image = cv2.circle(image, (int(x), int(y)), 5, (red,red,red), 2)
        #print(str(y) + " : " + str(x))
    dx = dx/count
    print(str(dx))
    adyacents = 1
    #Agrego los puntos extra??
    for i in range(1,len(corners)): #i (0,6) 
        ##GETTING ONLY de 6TH column
        if(i % 7 == 6):    #crazy ass math 
            xf, yf = corners[i][0]  # Extrae los valores
            xi, yi = corners[i-6][0]  # Extrae los valores
            image = cv2.circle(image, (int(xf), int(yf)), 5, (255,0,0), 5)
            vector = np.array([[xf-xi, yf-yi]])
            image = cv2.line(image, (int(xi), int(yi)), (int(xf), int(yf)), (0, 255, 0), 2)
            magnitude = np.linalg.norm(vector)
            direction = vector/magnitude #NORMALIZATION
            vector = np.array([[xf, yf]])
            newPoint = vector + direction *(dx*0.6)
            xNew, yNew = newPoint[0]
            image = cv2.circle(image, (int(xNew), int(yNew)), 2, (0,255,0), 2)
            vector = np.array([[xi, yi]])
            newPoint = vector - direction *(dx*0.6)
            xNew, yNew = newPoint[0]
            image = cv2.circle(image, (int(xNew), int(yNew)), 2, (0,255,0), 2)
    
    
    
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
    cv2.imshow('Chessboard',img)