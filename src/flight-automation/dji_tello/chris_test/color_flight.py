# BANSHEE UAV Robotics Team
# UAV Color Detection Script
import numpy as np
import time
import cv2
from djitellopy import Tello
from threading import Thread
import imutils

# Define Color Variables and Ranges
red_low = [20, 150, 150]
red_high = [40, 255, 255]
orange_low = []
orange_high = []
yellow_low = []
yellow_high = []
green_low = []
green_high = []
blue_low = []
blue_high = []
purple_low = []
purple_high = []

# A required callback method
def nothing(x):
    pass

# declare tello object and connect drone to python file
tello = Tello()
tello.connect()

# turn on stream
tello.streamon()

# print the first battery read
print(tello.get_battery())

# get first frame read (img)
frame_read = tello.get_frame_read()

# takeoff
tello.send_rc_control(0, 0, 0, 0)
tello.takeoff()

# create boolean land, which tells us when to land
land = False

# Default lock variables
x = 0
y = 0
z = 0
yaw = 0
a_id = -1

def fly_drone():
    global a_id, x, y, z, yaw, land

    match a_id:
        case 0:
            tello.send_rc_control(x, y, z, yaw)

        case 1:
            tello.move_up(100)
            tello.rotate_clockwise(360)
            tello.move_down(100)
        case 2:
            tello.flip('b')
            a_id = 0
        case 3:
            land = True
        case _:
            tello.send_rc_control(0, 0, 0, 0)
            pass

while not land:
    fly_thread = Thread(target=fly_drone())
    fly_thread.start()

    # checking to make 
    if not (tello.get_battery() == 0):
        # get frame
        img = frame_read.frame
        
        # Convert the BGR image to HSV image.
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
        # Set lower and upper range of HSV detection
        lower_range = np.array(red_low)
        upper_range = np.array([red_high])
        
        # Filter the image and get the binary mask, where white represents 
        # your target color
        mask = cv2.inRange(hsv, lower_range, upper_range)
        
        # Converting the binary mask to 3 channel image, this is just so 
        # we can stack it with the others
        mask_3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        # Find contours and label them on screen
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        # Loop for all contours
        contnum = 0
        # Dictionary for all contour information
        contdict = {}
        if len(contours) != 0:
            for c in contours:
                area = cv2.contourArea(c)
                # Only display contour for those having an area threshold of > 1000
                if area > 1000:
                    M = cv2.moments(c)
                    try:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                    except:
                        print("Contour not found!")
                    
                    contdict[str(contnum)] = {}
                    contdict[str(contnum)]['ID'] = contnum
                    contdict[str(contnum)]['area'] = area
                    contdict[str(contnum)]['cX'] = cX
                    contdict[str(contnum)]['cY'] = cY

                    cv2.drawContours(img, [c], -1, (0, 255, 0), 2)
                    cv2.circle(img, (cX, cY), 7, (0,0,0), -1)
                    cv2.putText(img, "ID: {}".format(contnum), (cX - 23, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
                    cv2.putText(img, "Location: ({}, {})".format(cX, cY), (450, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
                    contnum += 1        
            # Find smallest contour in dictionary and get its cX and cY value
            tempsize = 123456789
            location = 0
            for i in range(0, contnum):
                local_area = contdict[str(i)]['area']
                if local_area < tempsize:
                    location = i
                    tempsize = local_area
            try:
                cX = contdict[str(location)]['cX']
                cY = contdict[str(location)]['cY']
                # Display number of contours detected
                cv2.putText(img, "# of contours: {}".format(contnum), (600, 425), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
                cv2.putText(img, "Currently Tracking ID {}".format(contdict[str(location)]['ID']), (600, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
            except:
                # print("No Contours Detected.")
                pass

        # Show original image and the filtered image
        stacked = np.hstack((img, mask_3))
        cv2.imshow('Frames',cv2.resize(stacked,None,fx=0.75,fy=0.75))
        
        cv2.putText(img, str(tello.get_battery()), (100, 100), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0))

        # If the user presses q then exit the program
        key = cv2.waitKey(10)
        if key == ord('q'):
            land = True
            print("landing")
            tello.flip('f')
            tello.streamoff()

# Release the camera & destroy the windows, and land the drone    
cv2.destroyAllWindows()
tello.land()