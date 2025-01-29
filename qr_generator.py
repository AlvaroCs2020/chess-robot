import argparse
import imutils
import cv2
import sys
import numpy as np
import math
class CodeDetection():
    def __init__(self, initiImage):# load the input image from disk and resize it
        self.image = initiImage
        self.numberOfCodes = 3
        if self.image is None:
            print("[ERROR] No se pudo cargar la imagen. Verifica el path.")
            exit(1)
        self.centers = []
        self.markers = []
    def reset_centers(self):
        self.centers = []
        self.markers = []
    def set_image(self,im):
        self.image = im
    def get_image(self):
        return self.image
    
    def detection(self):
        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        arucoParams = cv2.aruco.DetectorParameters()

        detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams   )
        corners, ids, rejected = detector.detectMarkers(self.image)
        
        return corners, ids, rejected
    
    def proccesImage(self, corners, ids, rejected):
# verify *at least* one ArUco marker was detected
        if len(corners)> 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                    
                # draw the bounding box of the ArUCo detection
                cv2.line(self.image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(self.image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(self.image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(self.image, bottomLeft, topLeft, (0, 255, 0), 2)
                # compute and draw the center (x, y)-coordinates of the ArUco
                # marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(self.image, (cX, cY), 4, (0, 0, 255), -1)
                
                newElement = (markerID,(cX, cY))
                
                if markerID not in self.markers and markerID < self.numberOfCodes:
                    self.markers.append(markerID) 
                    self.centers.append(newElement)
                    #print("metimos"+ str(len(self.markers)))
                
                # draw the ArUco marker ID on the image
                cv2.putText(self.image, str(markerID),
                    (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
                #print("[INFO] ArUco marker ID: {}".format(markerID))
        return self.image 
    
    #Geometria y matematica
    def calculateDistance(p1, p2):
        return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
    
    def calculateAngles(self, triangle):
    # Extract points from the triangle
        if(len(triangle) != self.numberOfCodes):
            return
        try:
            A, B, C = triangle
        except:
            return
        def calculateDistance(p1, p2):
            return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
        # Calculate the sides of the triangle
        a = calculateDistance(B, C)  # Side opposite to vertex A
        b = calculateDistance(A, C)  # Side opposite to vertex B
        c = calculateDistance(A, B)  # Side opposite to vertex C

        # Use the law of cosines to calculate each angle
        angleA = math.degrees(math.acos((b**2 + c**2 - a**2) / (2 * b * c)))
        angleB = math.degrees(math.acos((a**2 + c**2 - b**2) / (2 * a * c)))
        angleC = 180 - angleA - angleB  # The sum of the angles in a triangle is 180°

        return angleA, angleB, angleC
    
    def geometryProcessing(self):
        
        if(len(self.markers) != self.numberOfCodes):
            pass

        self.centers.sort()

        for i in range(1,len(self.centers)):
            pt1,pt2 = self.centers[i-1][1],self.centers[i][1]  
            #print(str(pt1) + " "+ str(pt2)) 
            self.image = cv2.line(self.image, pt1,pt2, (255,0, 0), 2)
        triangle = []

        for (id,(x,y)) in self.centers:
            triangle.append((x,y))

        print(str(self.calculateAngles(triangle)))

    def show(self):
        # show the output image
        cv2.imshow("Image", self.image)
        cv2.waitKey(0)


#Test and example of the class above
def main():
    
    #delay = 1
    #window_name = 'OpenCV QR Code'

    qcd = cv2.QRCodeDetector()
    cap = cv2.VideoCapture(1)
    ret, frame = cap.read()
    
    while True:
        #cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        #codeDetector.set_image(frame)
        cv2.imshow("Image", frame)
        #corners, ids, rejected = codeDetector.detection()
        #codeDetector.proccesImage(corners, ids, rejected)
        #codeDetector.show()

        
camera_id = 0
delay = 1
window_name = 'OpenCV QR Code'

qcd = cv2.QRCodeDetector()
cap = cv2.VideoCapture(1)
ret, frame = cap.read()
codeDetector = CodeDetection(frame)
while not ret:
    print(ret)
    ret, frame = cap.read()
while True:
    ret, frame = cap.read()
    codeDetector.set_image(frame)
    corners, ids, rejected = codeDetector.detection()
    frame2 = codeDetector.proccesImage(corners, ids, rejected)
    codeDetector.set_image(frame2)
    codeDetector.geometryProcessing()
    codeDetector.reset_centers()
    if ret:
        cv2.imshow(window_name, codeDetector.get_image())
    else:
        cv2.imshow(window_name, frame)
    if cv2.waitKey(delay) & 0xFF == ord('r'):
        codeDetector.reset_centers()
    if cv2.waitKey(delay) & 0xFF == ord('q'):
        break

cv2.destroyWindow(window_name)
    


#main()
    