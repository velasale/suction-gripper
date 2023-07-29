# importing the modules
import cv2
import numpy as np

# set Width and Height of output Screen
frameWidth = 640
frameHeight = 360

# capturing Video from Webcam
cap = cv2.VideoCapture(4)

def drawSomething(img, width, height):
    color = (0, 0, 0) 
    x = int(width/2)
    y = int(height/2)
    # cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Draw a rectangular frame
    cv2.line(img, (x, 0), (x, height), color, 2)
    cv2.line(img, (0, y), (width, y), color, 2)



# running infinite while loop so that
# program keep running until we close it
while True:
    success, img = cap.read()
    imgResult = img.copy()

    # finding the colors for the points
    drawSomething(imgResult, frameWidth, frameHeight)        

    # displaying output on Screen
    cv2.imshow("Result", imgResult)



    key = cv2.waitKey(1)
    if key == 27:
        break