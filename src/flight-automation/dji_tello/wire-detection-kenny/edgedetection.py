import numpy as np
import cv2

#webcam is defined
webcam = cv2.VideoCapture(0, cv2.CAP_DSHOW)
#check for compatibility of the webcam
ret, frame = webcam.read()
print(frame)
try:
        if frame == None:
                webcam = cv2.VideoCapture(-1)
except:
                print("did not worked")

while(True):
        ret, frame = webcam.read()
        #opens the webcam and grabs the frame
        greyscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #greyscale
        blur = cv2.GaussianBlur(greyscale, (9,9),0)
        #Gaussian Blur
        canny = cv2.Canny(blur, 210, 90, 7, L2gradient = True)
        #edge detection
        ret, frame = cv2.threshold(canny, 90, 255, cv2.THRESH_BINARY)

        cv2.imshow('cam', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
                break

webcam.release()
cv2.destroyAllWindows()
        

