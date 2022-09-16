import cv2
import aruco_draw
import math
import pygame
from pygame.locals import *

cv2.namedWindow("preview")
vc = cv2.VideoCapture(0)
# need to change to a
x_P = 0.3
y_P = 0.3
lateral_P = 0.3
box_size = 200
land = False

while not land:
    # get frame
    rval, img = vc.read()

    # https://chev.me/arucogen/, useful to generate the ArUco codes
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    arucoParams = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict,
                                                       parameters=arucoParams)
    # cv2.aruco.drawDetectedMarkers(img, corners, ids)

    aruco_draw.centerloc(img, corners, ids)
    a_id = -1
    if not (ids is None):
        a_id = ids[0]
    text = ""
    match a_id:
        case 0:
            text = "AR Marker Dict 6x6, ID 0 - Follow the marker around, attempting to keep a static distance and " \
                   "angle away. This means finding the area and center of the AR marker and creating an algorithm to " \
                   "position the drone a set distance depending on the location of the AR marker. "
            average_x_position = 0
            average_y_position = 0
            corner = corners[0]
            for a_corner in corner[0]:
                print("acorner" + str(a_corner))
                average_x_position += a_corner[0]
                average_y_position += a_corner[1]
            the_corner = corner[0]
            bottom_left_corner = the_corner[0]
            top_right_corner = the_corner[2]
            top_left_corner = the_corner[1]
            bottom_right_corner = the_corner[3]
            print(bottom_left_corner)
            print(top_right_corner)
            print(top_left_corner)
            print(bottom_right_corner)
            diagonal1 = math.sqrt(
                (bottom_left_corner[0] + top_left_corner[0]) ** 2 + (bottom_left_corner[1] + top_left_corner[1]) ** 2)
            diagonal2 = math.sqrt(
                (top_left_corner[0] + bottom_right_corner[0]) ** 2 + (top_left_corner[1] + bottom_right_corner[1]) ** 2)
            area = diagonal2 * diagonal1 / 2

            height, width = img.shape[:2]
            average_x_position /= 4
            average_y_position /= 4
            average_y_position = int(average_y_position)
            y_center = height / 2
            y_center = int(y_center)
            y_distance = average_y_position - y_center
            cv2.line(img, (0, y_center), (width, y_center), (0, 255, 0), 10)
            cv2.line(img, (0, average_y_position), (width, average_y_position), (0, 0, 255), 10)
            average_x_position = int(average_x_position)
            x_center = width / 2
            x_center = int(x_center)
            x_distance = average_x_position - x_center
            cv2.line(img, (x_center, 0), (x_center, height), (0, 255, 0), 10)
            cv2.line(img, (average_x_position, 0), (average_x_position, height), (0, 0, 255), 10)
            half_box_size = int(box_size / 2)
            cv2.rectangle(img, (x_center - half_box_size, y_center - half_box_size),
                          (x_center + half_box_size, y_center + half_box_size), (255, 0, 0), 10)
            x_movement = int(x_distance * x_P)
            y_movement = int(y_distance * y_P)

        case 1:
            text = "AR Marker Dict 6x6, ID 1 - Increase height of drone to 250cm above its current location, " \
                   "scan surroundings in a 360 degree radius, and then return back to the original position. (Further " \
                   "integration of this task will be with Comms team and the powerline detection script) "
        case 2:
            text = "AR Marker Dict 6x6, ID 2 - Do a flip (Just for fun, you can choose what tricks the drone does)"

        case 3:
            text = "AR Marker Dict 6x6, ID 3 - Land in a safe location"
        case _:
            text = "No marker detected, spin in a 360 circle until marker detected"
    # show image
    cv2.putText(img, text, (100, 100), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0))

    cv2.imshow("drone", img)
    key = cv2.waitKey(10)
    if key == ord('q'):
        land = True
        print("landing")
cv2.destroyAllWindows()

