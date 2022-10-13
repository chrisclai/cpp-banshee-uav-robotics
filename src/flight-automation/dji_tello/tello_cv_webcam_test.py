from threading import Thread

from djitellopy import Tello
import cv2
import aruco_draw
from simple_pid import PID
import json
import time

# import constants and define
f = open('constants.json')

constants = json.load(f)
x_P = constants['x_P']
x_I = constants['x_I']
x_D = constants['x_D']
y_P = constants['y_P']
y_I = constants['y_I']
y_D = constants['y_D']
z_P = constants['z_P']
z_I = constants['z_I']
z_D = constants['z_D']

# box size is the size of the target box's length, area of the box = box_size * box_size
box_size = 50

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

# create the x & y axis pid's
# PID comes from simple-pid, an online api
x_pid = PID(x_P, x_I, x_D, setpoint=1)
y_pid = PID(y_P, y_I, y_D, setpoint=1)
z_pid = PID(z_P, z_I, z_D, setpoint=1)
# set the limits
x_pid.output_limits = (-50, 50)
y_pid.output_limits = (-50, 50)
z_pid.output_limits = (-50, 50)

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
            tello.flip()
        case 3:
            land = True
        case _:
            tello.send_rc_control(0, 0, 0, 0)
            pass

    # print("speed (x,y,z): " + str(tello.get_speed_x()) + ", " + str(tello.get_speed_y()) + ", " + str(
    #    tello.get_speed_z()))
    # print("yaw: " + str(tello.get_yaw()))
    # print("pitch: " + str(tello.get_pitch()))
    # print("roll: " + str(tello.get_roll()))
    # print("battery status: " + str(tello.get_battery()))
    # print("altitude: " + str(tello.get_height()))
    # print("barometer:" + str(tello.get_barometer()))


# main loop to run code
times = []
points = []
box_size_array = []

while not land:
    fly_thread = Thread(target=fly_drone())
    fly_thread.start()

    # checking to make 
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

                # turn on the pid capture data
                x_pid.auto_mode = True
                y_pid.auto_mode = True
                z_pid.auto_mode = True
                # update constants
                f = open('constants.json')

                constants = json.load(f)
                try:
                    x_P = constants['x_P']
                    x_I = constants['x_I']
                    x_D = constants['x_D']
                    y_P = constants['y_P']
                    y_I = constants['y_I']
                    y_D = constants['y_D']
                    z_P = constants['z_P']
                    z_I = constants['z_I']
                    z_D = constants['z_D']

                    x_pid.tunings = (x_P, x_I, x_D)
                    y_pid.tunings = (y_P, y_I, y_D)
                    z_pid.tunings = (z_P, z_I, z_D)
                except:
                    print("values not correct")
                # get average positions of the x and z positions
                average_x_position = 0
                average_z_position = 0
                corner = corners[0]
                for a_corner in corner[0]:
                    average_x_position += a_corner[0]
                    average_z_position += a_corner[1]
                average_x_position /= 4
                average_z_position /= 4
                average_x_position = int(average_x_position)
                average_z_position = int(average_z_position)

                # get the corners of the id, and split into the x and z values
                # find the area

                area = cv2.contourArea(corners[0])

                # draw the red and green lines, along with the blue boxes
                height, width = img.shape[:2]
                z_center = height / 2
                z_center = int(z_center)
                cv2.line(img, (0, z_center), (width, z_center), (0, 255, 0), 10)
                cv2.line(img, (0, average_z_position), (width, average_z_position), (0, 0, 255), 10)

                x_center = width / 2
                x_center = int(x_center)

                cv2.line(img, (x_center, 0), (x_center, height), (0, 255, 0), 10)
                cv2.line(img, (average_x_position, 0), (average_x_position, height), (0, 0, 255), 10)
                half_box_size = int(box_size / 2)
                cv2.rectangle(img, (x_center - half_box_size, z_center - half_box_size),
                              (x_center + half_box_size, z_center + half_box_size), (255, 0, 0), 10)

                # perform calculations
                x_distance = average_x_position - x_center
                y_distance = (area - box_size * box_size) / 100
                z_distance = average_z_position - z_center
                final_x_movement = x_pid(x_distance)
                final_x_movement = -int(final_x_movement)
                final_z_movement = z_pid(z_distance)
                final_z_movement = int(final_z_movement)
                # print("area" + str(area))
                # print("box_size" + str(box_size * box_size))
                # print(y_distance)
                final_y_movement = y_pid(y_distance)
                final_y_movement = int(final_y_movement)
                x = final_x_movement
                y = final_y_movement
                z = final_z_movement
                yaw = 0

            case 1:
                text = "AR Marker Dict 6x6, ID 1 - Increase height of drone to 250cm above its current location, " \
                       "scan surroundings in a 360 degree radius, and then return back to the original position. (" \
                       "Further " \
                       "integration of this task will be with Comms team and the powerline detection script) "

            case 2:
                text = "AR Marker Dict 6x6, ID 2 - Do a flip (Just for fun, you can choose what tricks the drone does)"

                break
            case 3:
                text = "AR Marker Dict 6x6, ID 3 - Land in a safe location"
                # tello.land()
                land = True

            case _:
                text = "No marker detected, spin in a 360 circle until marker detected"

                x = 0
                y = 0
                z = 0
                yaw = 0
                x_pid.auto_mode = False
                y_pid.auto_mode = False
                z_pid.auto_mode = False

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

cv2.destroyAllWindows()

tello.land()
