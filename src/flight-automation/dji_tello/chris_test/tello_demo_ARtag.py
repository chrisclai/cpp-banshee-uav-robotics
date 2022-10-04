from threading import Thread

from djitellopy import Tello
import cv2
import json
import time
import aruco_detect

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
tello.move_up(20)

# create boolean land, which tells us when to land
land = False

# this calculates the area using the shoelace theorem, from a stackoverflow post
x = 0
y = 0
z = 0
yaw = 0
a_id = -1


def fly_drone():
    global a_id, x, y, z, yaw, land

    match a_id:
        case 0:
            tello.rotate_clockwise(360)
        case 1:
            tello.move_up(20)
            tello.move_down(20)
            tello.move_up(20)
            tello.move_down(20)
            tello.move_up(20)
            tello.move_down(20)
        case 2:
            tello.flip('f')
            tello.flip('b')
            tello.flip('l')
            tello.flip('r')
        case 3:
            land = True
        case _:
            pass

while not land:
    fly_thread = Thread(target=fly_drone())
    fly_thread.start()

    # checking to make sure battery is not too low
    if not (tello.get_battery() == 0):

        # get frame
        img = frame_read.frame

        # Find AR Markers
        try:
            corners, ids = aruco_detect.findArucoMarkers(img, 6, 50)
            aruco_detect.centerloc(img, corners, ids)
        except:
            print("No image!")

        a_id = -1
        if not (ids is None):
            a_id = ids[0]
        text = ""
        
        match a_id:
            case 0:
                text = "State 0 detected. Rotating 360 degrees!"
            case 1:
                text = "State 1 detected. Bounce action initiated"
            case 2:
                text = "State 2 detected. Sequence of flips initiated"
            case 3:
                text = "State 3 detected. Landing the drone!"
                land = True
            case _:
                text = "No marker detected!"

        # show image
        cv2.putText(img, text, (50, 75), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 3)
        cv2.putText(img, f"Tello Battery: {str(tello.get_battery())}%", (50, 650), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 3)

        cv2.imshow("drone", img)
        key = cv2.waitKey(10)
        if key == ord('q'):
            land = True
            print("landing")
            tello.streamoff()

cv2.destroyAllWindows()

tello.land()
