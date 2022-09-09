from djitellopy import Tello
import cv2
import aruco_draw

tello = Tello()

tello.connect()

# https://djitellopy.readthedocs.io/en/latest/tello/#djitellopy.tello.Tello.connect_to_wifi
# not sure if the following line needs to be included:
# tello.connect_to_wifi(ssid, password)
tello.streamon()

frame_read = tello.get_frame_read()

# need to change to a
key = cv2.waitKey(1)

tello.takeoff()

while key != ord('q'):
    # get frame
    img = frame_read.frame

    # https://chev.me/arucogen/, useful to generate the ArUco codes
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    arucoParams = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict,
                                                       parameters=arucoParams)
    aruco_draw.centerloc(img, corners, ids)
    a_id = -1
    if not (ids is None):
        a_id = ids[0]

    match a_id:
        case 0:
            print("AR Marker Dict 6x6, ID 0 - Follow the marker around, attempting to keep a static distance and "
                  "angle away. This means finding the area and center of the AR marker and creating an algorithm to "
                  "position the drone a set distance depending on the location of the AR marker.")

        case 1:
            print("AR Marker Dict 6x6, ID 1 - Increase height of drone to 250cm above its current location, "
                  "scan surroundings in a 360 degree radius, and then return back to the original position. (Further "
                  "integration of this task will be with Comms team and the powerline detection script)")
            tello.move_up(250)
            tello.rotate_clockwise(360)
            tello.move_down(250)
        case 2:
            print("AR Marker Dict 6x6, ID 2 - Do a flip (Just for fun, you can choose what tricks the drone does)")
            tello.flip_forward()
        case 3:
            print("AR Marker Dict 6x6, ID 3 - Land in a safe location")
            tello.land()
        case _:
            print("No marker detected, spin in a 360 circle until marker detected")

    # show image
    cv2.imshow("drone", img)
    key = cv2.waitKey(1)
    # telemetry print
    # from instructions:
    # Your script should always be reading the following sensor data from the tello:
    # - Drone Directions: Throttle, Yaw, Pitch, Roll
    # - Battery status
    # - Packet transfer status (or connection status)
    # - Altitude status
    # - Barometer status
    # - Any other ToF (Time of Flight) data that can be read from the API

    cv2.putText(img, text, (100,100),cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0))
    print("speed (x,y,z): " + str(tello.get_speed_x()) + ", " + str(tello.get_speed_y()) + ", " + str(
        tello.get_speed_z()))
    print("yaw: " + str(tello.get_yaw()))
    print("pitch: " + str(tello.get_pitch()))
    print("roll: " + str(tello.get_roll()))
    print("battery status: " + str(tello.get_battery()))
    print("altitude: " + str(tello.get_height()))
    print("barometer:" + str(tello.get_barometer()))
