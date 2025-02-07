# import required libraries
import cv2
import numpy as np
import time
cap = cv2.VideoCapture(1)
ret, frame = cap.read()
doOnlyOnce = True
first_corners = []
slicesFst = []
def extract_region(image, points):
    """
    Extracts a region from the image based on four given points.
    
    Parameters:
        image (numpy.ndarray): The input image.
        points (list of tuples): Four points (x, y) defining the region.
    
    Returns:
        numpy.ndarray: The extracted region.
    """
    # Define the destination size based on the bounding box of the points
    width = int(max(np.linalg.norm(points[0] - points[1]), np.linalg.norm(points[2] - points[3])))
    height = int(max(np.linalg.norm(points[0] - points[3]), np.linalg.norm(points[1] - points[2])))
    
    dst_points = np.array([[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]], dtype=np.float32)
    
    # Compute the perspective transformation matrix
    matrix = cv2.getPerspectiveTransform(np.array(points, dtype=np.float32), dst_points)
    
    # Apply the transformation
    extracted_region = cv2.warpPerspective(image, matrix, (width, height))
    
    return extracted_region
def saveSlices(image, corners):
    slices = []
    coordinates = []
    for i in range(0,len(corners)-10):
        #points = np.array([(100, 100), (400, 100), (400, 300), (100, 300)])
        x1, y1 = corners[i][0]
        x2, y2 = corners[i+1][0]
        x3, y3 = corners[i+9][0]
        x4, y4 = corners[i+10][0]
        points = np.array([(x3, y3), (x4, y4), (x2, y2), (x1, y1)])
        extracted = extract_region(image, points)
        slices.append(extracted)
        coordinates.append(points)
    return slices, coordinates
        

def addCorners(corners, image):
    newCorners = np.empty((0, 1, 2)) 

    
    red = 50
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

    #Agrego los puntos extra??
    for i in range(1,len(corners)): #i (0,6) 
        ##GETTING ONLY de 6TH column
        if(i % 7 == 6):    #crazy ass math 
            xf, yf = corners[i][0]  # Extrae los valores
            xi, yi = corners[i-6][0]  # Extrae los valores
            #image = cv2.circle(image, (int(xf), int(yf)), 5, (255,0,0), 5)
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
            #print("len "+str(len(slice_arr)))           
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
        #print(np.linalg.norm(direction))
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
        #image = cv2.line(image, (int(xi), int(yi)), (int(xf), int(yf)), (0, 255, 0), 2)
        
        vector = np.array([[xf-xi, yf-yi]]) 
        
        magnitude = np.linalg.norm(vector)
        
        direction = vector/magnitude #NORMALIZATION
        vector = np.array([[xf, yf]]) 
        newPoint = vector + direction *(dx*0.6)
        xNew, yNew = newPoint[0]
        lastSlice[i][0] = xNew, yNew 

    newCorners = np.concatenate((fstSlice,newCorners, lastSlice), axis=0)
    #
    #COUNTER_TEST = (COUNTER_TEST + 1) %len(newCorners) 
    #
    #for i in range(0,COUNTER_TEST): #i (0,6) 
    #    x, y = newCorners[i][0]  # Extrae los valores
    #    image = cv2.circle(image, (int(x), int(y)), 5, (255,255,red), 2)
    #
    ##print(len(newCorners))
    ##print(newCorners)
    ###PUNTOS FINALES E INICIALES
    #x, y = newCorners[0][0]  # Extrae los valores
    #image = cv2.circle(image, (int(x), int(y)), 8, (0,0,120), 2)
    #x, y = newCorners[len(newCorners)-1][0]  # Extrae los valores
    #image = cv2.circle(image, (int(x), int(y)), 8, (0,0,255), 2)
    
    return image, newCorners 
def getBiggestError(slices, coordinates, img):
    #get slices
    max_score = 0
    for i in range(0,len(slices)-1):
        extracted = extract_region(img, coordinates[i])
        
        gray1 = cv2.cvtColor(extracted, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(slices[i], cv2.COLOR_BGR2GRAY)
        
        mse_score = int(np.mean((extracted.astype("float") - slices[i].astype("float")) ** 2))
        if (mse_score > max_score):
            max_score = mse_score
            index = i
        print(str(i) + " error: " +str(mse_score))
    return index
staticSlices = []
staticCoordinates = []
index = 0

while not ret:
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
            _, staticCorners = addCorners(first_corners, img)
            staticSlices, staticCoordinates = saveSlices(img, staticCorners)#TODO hacer la comparacion frame por frame
            doOnlyOnce = False
            
        # Draw and display the corners
        #img = cv2.drawChessboardCorners(img, (n,n), corners,ret)
    if not doOnlyOnce:
       
        
        img, _ = addCorners(first_corners, img)
        point = staticCoordinates[index]
        extractedOld = staticSlices[index]
        extracted = extract_region(img, point)
        
        gray1 = cv2.cvtColor(extracted, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(extractedOld, cv2.COLOR_BGR2GRAY)
        
        mse_score = int(np.mean((gray1.astype("float") - gray2.astype("float")) ** 2))
        #print("Erro medio cuadratico "+str(mse_score))
        topRight = (int(point[0][0]), int(point[0][1]))
        bottomRight = (int(point[1][0]), int(point[1][1]))
        bottomLeft = (int(point[2][0]), int(point[2][1]))
        topLeft = (int(point[3][0]), int(point[3][1]))
        # draw the bounding box of the ArUCo detection
        # points = np.array([(x1, y1), (x2, y2), (x3, y3), (x4, y4)])
        if cv2.waitKey(1) & 0xFF == ord('r'):
            index = getBiggestError(staticSlices,staticCoordinates,img)
        cv2.line(img, topLeft, topRight, (0, 255, 0), 2)
        cv2.line(img, topRight, bottomRight, (0, 255, 0), 2)
        cv2.line(img, bottomRight, bottomLeft, (0, 255, 0), 2)
        cv2.line(img, bottomLeft, topLeft, (0, 255, 0), 2)
        
        cv2.imshow('ext',extracted)
        
        #time.sleep(0.1)
        
    cv2.imshow('Chessboard',img)