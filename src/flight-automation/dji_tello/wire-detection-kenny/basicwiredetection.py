import numpy as np
import cv2
import imutils
import time

# Define webcam used
webcam = cv2.VideoCapture(0, cv2.CAP_DSHOW)
# Check for compatibility. If not, force search webcam
ret, frame = webcam.read()
print(frame)
try:
    if frame == None:
        webcam = cv2.VideoCapture(-1)
except:
    print("Task Failed Successfully. Move on.")

while(True):
    # Grabbing frame from webcam
    ret, frame = webcam.read()

    # Apply Grayscale
    grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian Blur
    blur = cv2.GaussianBlur(grayscale, (9,9), 0)

    # Create Mask/Threshold
    threshold = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 23, 3)

    #converting BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    #define range of red color in HSV
    lower_red = np.array([30,150,50])
    upper_red = np.array([255,255,180])
    
    # create a red HSV colour boundary and
    # threshold HSV image
    mask = cv2.inRange(hsv, lower_red, upper_red)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)
    #Finding the contour of the wire
    cnts = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    for c in cnts:
        area = cv2.contourArea(c)
        length = cv2.arcLength(c,True)
        edges = cv2.Canny(frame, 200, 200)
        #frameWidth = (int(webcam.get(3)))
        #frameHeight = (int(webcam.get(4)))
        if ( 3000 > area > 2400) and (length > 900):
            cv2.drawContours(frame, [c], -1, (36,255,12), 1)
            #pass
            cv2.imshow('frame', frame)
            cv2.imshow('edge', edges)
            cv2.putText(frame,"Wire",(150,200),cv2.FONT_HERSHEY_SIMPLEX,.75,(255,0,0),2)

    cv2.imshow('video', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

webcam.release()
cv2.destroyAllWindows()