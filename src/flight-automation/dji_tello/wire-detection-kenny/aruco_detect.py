import numpy as np
import cv2
import cv2.aruco as aruco
import imutils
import time

def findArucoMarkers(img, markerSize, totalMarkers, draw=True):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    corners, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters = arucoParam)
    return corners, ids

def centerloc(frame, corners, ids):
    for (markerCorner, markerID) in zip(corners, ids):
        corners = markerCorner.reshape((4,2))
        topLeft, topRight, bottomRight, bottomLeft = corners

        # convert each of the (x, y)-coordinate pairs to integers
        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))

        # draw the bounding box of the ArUCo detection
        cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
        cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
        cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
        cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
        # compute and draw the center (x, y)-coordinates of the
        # ArUco marker
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
        # draw the ArUco marker ID on the frame
        cv2.putText(frame, f"({cX},{cY})", (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)