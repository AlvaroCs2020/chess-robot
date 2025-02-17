import cv2
import numpy as np
import time

class BoardProcessor():
    def __init__(self):
        self.emptySlices = []
        self.doOnlyOnce = True
    def setEmptySlices(self,slices):
        self.emptySlices = slices
    def extract_region(self,image, points):
        """
        Extracts a region from the image based on four given points and applies a mask to black out edges,
        keeping only the center portion visible.
        
        Parameters:
            image (numpy.ndarray): The input image.
            points (list of tuples): Four points (x, y) defining the region.
        
        Returns:
            numpy.ndarray: The extracted region with masked edges, highlighting the center.
        """
        # Define the destination size based on the bounding box of the points
        width = int(max(np.linalg.norm(points[0] - points[1]), np.linalg.norm(points[2] - points[3])))
        height = int(max(np.linalg.norm(points[0] - points[3]), np.linalg.norm(points[1] - points[2])))
        
        dst_points = np.array([[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]], dtype=np.float32)
        
        # Compute the perspective transformation matrix
        matrix = cv2.getPerspectiveTransform(np.array(points, dtype=np.float32), dst_points)
        
        # Apply the transformation
        extracted_region = cv2.warpPerspective(image, matrix, (width, height))
        
        # Create a mask with a gradient effect, keeping the center visible
        mask = np.zeros((height, width), dtype=np.uint8)
        center = (width // 2, height // 2)
        radius = min(width, height) // 3  # Define a radius for the central area
        cv2.circle(mask, center, radius, 255, -1)
        #mask = cv2.GaussianBlur(mask, (51, 51), 30)
        
        # Apply mask to extracted region
        extracted_region = cv2.bitwise_and(extracted_region, extracted_region, mask=mask)
        
        return extracted_region
    def saveSlices(self,image, corners):
        
        self.lastSavedFrame = image
        slices = []
        coordinates = []
        for i in range(1,9): #hasta la penultima fila
            for j in range(0,8): #hasta la penultima columna
                x1, y1 = corners[j+9*(i-1)][0]
                x2, y2 = corners[j+1+9*(i-1)][0]
                x3, y3 = corners[j+9*i][0]
                x4, y4 = corners[(j+1) + 9*i][0]
                points = np.array([(x3, y3), (x4, y4), (x2, y2), (x1, y1)])
                extracted = self.extract_region(image, points)
                slices.append(extracted)
                coordinates.append(points) 

        return slices, coordinates
# Math ugliest shi
    def addCorners(self, corners, image):
        newCorners = np.empty((0, 1, 2)) 
        dx = 0
        count = 0

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
        dx = dx/count
        #Agrego los primeros puntos
        for i in range(1,len(corners)): #i (0,6) 
            ##GETTING ONLY de 6TH column
            if(i % 7 == 6):    #crazy ass math 
                xf, yf = corners[i][0]  # Extrae los valores
                xi, yi = corners[i-6][0]  # Extrae los valores
                
                vector = np.array([[xf-xi, yf-yi]])
                magnitude = np.linalg.norm(vector)
                direction = vector/magnitude #NORMALIZATION
                vector = np.array([[xf, yf]])
                newPoint = vector + direction *(dx*0.6)
                xNew, yNew = newPoint[0]
                
                prependElement = np.array([[[xNew, yNew]]])  # Shape (1, 1, 2)
                vector = np.array([[xi, yi]])
                newPoint = vector - direction *(dx*0.6)
                xNew, yNew = newPoint[0]
                
                appendElement = np.array([[[xNew, yNew]]])  # Shape (1, 1, 2)
                
                slice_arr = corners[i-6:i+1]  # Shape (n, 1, 2)
                slice_arr = np.concatenate((appendElement, slice_arr, prependElement), axis=0)               
                newCorners = np.concatenate((newCorners,slice_arr), axis=0)

        #lo mismo pero con los primeros y ultimos elementos de la lista!!!
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
        
        return image, newCorners
    def getBiggestError(self, slices, coordinates, img):
        if self.emptySlices == []:
            return 0
        #get slices
        indexs = []
        for i in range(0,len(slices)):
            extracted = self.extract_region(img, coordinates[i])
            
            gray1 = cv2.cvtColor(extracted, cv2.COLOR_BGR2GRAY)
            gray2 = cv2.cvtColor(slices[i], cv2.COLOR_BGR2GRAY)
            
            mse_score = int(np.mean((gray1.astype("float") - gray2.astype("float")) ** 2))
            indexs.append((i,int(mse_score)))
            
        
        #Obtenemos los dos MSE mas altos
        indexs = sorted(indexs, key=lambda x: x[1])
        
        index = indexs[len(indexs) - 1][0]
        lastIndex = indexs[len(indexs) - 2][0]
        
        #Ahora, determinamos cual de los dos cuadrados es mas probable que este vacio
        #Si estamos moviendo una pieza y buscamos saber a donde fue, no deberiamos devovler un indice vacio
        gray1 = cv2.cvtColor(self.emptySlices[index], cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(slices[index], cv2.COLOR_BGR2GRAY)
        mseA_score = int(np.mean((gray1.astype("float") - gray2.astype("float")) ** 2))
        
        gray1 = cv2.cvtColor(self.emptySlices[lastIndex], cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(slices[lastIndex], cv2.COLOR_BGR2GRAY)
        mseB_score = int(np.mean((gray1.astype("float") - gray2.astype("float")) ** 2))
        #devolvemos el que sea MAS distinto de la foto vacia
        if mseA_score > mseB_score: 
            index = lastIndex
        return index


#DEMO

def main():
    boardProcessor = BoardProcessor()
    cap = cv2.VideoCapture(1)
    doOnlyOnce = True
    ret, img = cap.read()    
    while not ret:
        print("[Update latest squaresssnged]")
        ret, img = cap.read()
    while True:
        ret, img = cap.read()
        original = img.copy()
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (7,7),None)

        if not ret:
            break
        
        if doOnlyOnce:
          _, staticCorners = boardProcessor.addCorners(corners, img)
          boardProcessor.setEmptySlices(img)
          doOnlyOnce = False
        
        key = cv2.waitKey(1)
        points = np.array([(100, 200), (200,200), (200, 300), (100, 300)])
        
        if key & 0xFF == ord('r'):
            print("[Update latest square changed]")

        cv2.imshow("extraction example",boardProcessor.extract_region(img, points))
        cv2.imshow("demo class", original)
        
main()