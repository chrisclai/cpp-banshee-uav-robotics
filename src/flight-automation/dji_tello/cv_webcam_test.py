import numpy
import cv2
import aruco_draw
cv2.namedWindow("preview")
vc = cv2.VideoCapture(0)
# need to change to a
key = cv2.waitKey(1)


while key != ord('q'):
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
    cv2.putText(img, text, (100,100),cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0))

    cv2.imshow("drone", img)
    key = cv2.waitKey(1)
    cv2.destroyAllWindows()
