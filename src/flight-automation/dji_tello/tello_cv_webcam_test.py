import math
import numpy as np
from djitellopy import Tello
import cv2
import aruco_draw
from simple_pid import PID
import json

# constants
f = open('constants.json')

constants = json.load(f)
x_P = constants['x_P']
x_I = constants['x_I']
x_D = constants['x_D']
y_P = constants['y_P']
y_I = constants['y_I']
y_D = constants['y_D']
box_size = 100

# connect tello drone to python file
tello = Tello()

tello.connect()

tello.streamon()

print(tello.get_battery())

frame_read = tello.get_frame_read()

tello.send_rc_control(0, 0, 0, 0)
tello.takeoff()
land = False

x_pid = PID(x_P, x_I, x_D, setpoint=1)
y_pid = PID(y_P, y_I, y_D, setpoint=1)

x_pid.output_limits = (-20, 20)
y_pid.output_limits = (-20, 20)


def PolyArea(x, y):
    return 0.5 * np.abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))


while not land:

    if not (tello.get_battery() == 0):

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
        text = ""
        match a_id:
            case 0:

                text = "AR Marker Dict 6x6, ID 0 - Follow the marker around, attempting to keep a static distance and " \
                       "angle away. This means finding the area and center of the AR marker and creating an algorithm to " \
                       "position the drone a set distance depending on the location of the AR marker. "
                x_pid.auto_mode = True
                y_pid.auto_mode = True
                f = open('constants.json')

                constants = json.load(f)
                x_P = constants['x_P']
                x_I = constants['x_I']
                x_D = constants['x_D']
                y_P = constants['y_P']
                y_I = constants['y_I']
                y_D = constants['y_D']
                x_pid.tunings = (x_P, x_I, x_D)
                y_pid.tunings = (y_P, y_I, y_D)
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
                x_values = [bottom_right_corner[0], top_right_corner[0], top_left_corner[0], bottom_right_corner[0]]
                y_values = [bottom_right_corner[1], top_right_corner[1], top_left_corner[1], bottom_right_corner[1]]

                area = PolyArea(x_values, y_values)

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

                y_distance = area - box_size * box_size
                final_x_movement = x_pid(x_distance)

                final_x_movement = -int(final_x_movement)

                final_y_movement = y_pid(y_distance)
                final_y_movement = -int(final_y_movement)

                tello.send_rc_control(final_x_movement, final_y_movement, 0, 0)
                print("area:" + str(area))
                print("y_distance:" + str(y_distance))

            case 1:
                text = "AR Marker Dict 6x6, ID 1 - Increase height of drone to 250cm above its current location, " \
                       "scan surroundings in a 360 degree radius, and then return back to the original position. (" \
                       "Further " \
                       "integration of this task will be with Comms team and the powerline detection script) "
                # tello.move_up(100)
                # tello.rotate_clockwise(360)
                # tello.move_down(100)
            case 2:
                text = "AR Marker Dict 6x6, ID 2 - Do a flip (Just for fun, you can choose what tricks the drone does)"
                # tello.flip_forward()
            case 3:
                text = "AR Marker Dict 6x6, ID 3 - Land in a safe location"
                # tello.land()
                land = True
            case _:
                text = "No marker detected, spin in a 360 circle until marker detected"

                tello.send_rc_control(0, 0, 0, 0)
                x_pid.auto_mode = False
                y_pid.auto_mode = False
        # show image
        cv2.putText(img, text, (100, 100), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0))

        cv2.imshow("drone", img)
        key = cv2.waitKey(10)
        if key == ord('q'):
            land = True
            print("landing")
            tello.streamoff()

        # telemetry print
        # from instructions:
        # Your script should always be reading the following sensor data from the tello:
        # - Drone Directions: Throttle, Yaw, Pitch, Roll
        # - Battery status
        # - Packet transfer status (or connection status)
        # - Altitude status
        # - Barometer status
        # - Any other ToF (Time of Flight) data that can be read from the API

        print("speed (x,y,z): " + str(tello.get_speed_x()) + ", " + str(tello.get_speed_y()) + ", " + str(
            tello.get_speed_z()))
        print("yaw: " + str(tello.get_yaw()))
        print("pitch: " + str(tello.get_pitch()))
        print("roll: " + str(tello.get_roll()))
        print("battery status: " + str(tello.get_battery()))
        print("altitude: " + str(tello.get_height()))
        print("barometer:" + str(tello.get_barometer()))
cv2.destroyAllWindows()

tello.land()
