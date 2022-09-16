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
x_P = 0.3
y_P = 0.3
lateral_P = 0.3
box_size = 200
tello.send_rc_control(0, 0, 0, 0)
print(tello.get_battery())
tello.takeoff()
land = False


async def g():
    # Pause here and come back to g() when f() is ready
    r = await input("press q to land")
    if r == "q":
        land = True
        tello.land()
    return 0


while key != ord('q') and land is False:

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
                average_x_position = 0
                average_y_position = 0
                corner = corners[0]
                for a_corner in corner[0]:
                    print("acorner" + str(a_corner))
                    average_x_position += a_corner[0]
                    average_y_position += a_corner[1]
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
                print(x_movement)
                final_x_movement = 0
                final_y_movement = 0
                if x_movement > 10:
                    final_x_movement = 10
                    pass
                elif x_movement < -10:
                    final_x_movement = -10
                    pass

                tello.send_rc_control(final_x_movement, -2, 0, 0)

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

                # tello.rotate_clockwise(1)
                tello.send_rc_control(0, 0, 0, 0)

        # show image
        cv2.putText(img, text, (100, 100), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0))
        cv2.imshow("drone", img)

        key = cv2.waitKey(1)
        if key == ord('q'):
            tello.land()
            land = True

        cv2.destroyAllWindows()
        # telemetry print
        # from instructions:
        # Your script should always be reading the following sensor data from the tello:
        # - Drone Directions: Throttle, Yaw, Pitch, Roll
        # - Battery status
        # - Packet transfer status (or connection status)
        # - Altitude status
        # - Barometer status
        # - Any other ToF (Time of Flight) data that can be read from the API

        # print("speed (x,y,z): " + str(tello.get_speed_x()) + ", " + str(tello.get_speed_y()) + ", " + str(
        #    tello.get_speed_z()))
        print("yaw: " + str(tello.get_yaw()))
        # print("pitch: " + str(tello.get_pitch()))
        # print("roll: " + str(tello.get_roll()))
        # print("battery status: " + str(tello.get_battery()))
        # print("altitude: " + str(tello.get_height()))
        # print("barometer:" + str(tello.get_barometer()))
        g()
tello.emergency()
