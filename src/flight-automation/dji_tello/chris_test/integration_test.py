from djitellopy import Tello
import cv2
import pygame
import numpy as np
import time
import aruco_detect
import json
from simple_pid import PID
# Speed of the drone
S = 60
# Frames per second of the pygame window display
FPS = 120

f = open('../constants.json')

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

# create the x & y axis pid's
# PID comes from simple-pid, an online api
x_pid = PID(x_P, x_I, x_D, setpoint=1)
y_pid = PID(y_P, y_I, y_D, setpoint=1)
z_pid = PID(z_P, z_I, z_D, setpoint=1)
# set the limits
x_pid.output_limits = (-50, 50)
y_pid.output_limits = (-50, 50)
z_pid.output_limits = (-50, 50)

box_size = 50

class TelloUI(object):
    """ Maintains the Tello display and moves it through the keyboard keys.
        Press escape key to quit.
        The controls are:
            - T: Takeoff
            - L: Land
            - Arrow keys: Forward, backward, left and right.
            - A and D: Counter clockwise and clockwise rotations (yaw)
            - W and S: Up and down.
            - U: Flip forward
            - H: Flip left
            - J: Flip back
            - K: Flip right
    """

    def __init__(self):
        # Initialize pygame
        pygame.init()

        # Create pygame window
        pygame.display.set_caption("TelloTV Live!")
        self.screen = pygame.display.set_mode([960, 720])

        # Init Tello object that interacts with the Tello drone
        self.tello = Tello()

        # Drone velocities between -100~100
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 10

        # Self State
        self.a_id = -1

        self.send_rc_control = False
        self.x = 0
        self.y = 0
        self.z = 0
        self.yaw = 0
        # create update timer
        pygame.time.set_timer(pygame.USEREVENT + 1, 1000 // FPS)

    def run(self):

        self.tello.connect()
        self.tello.set_speed(self.speed)

        # In case streaming is on. This happens when we quit this program without the escape key.
        self.tello.streamoff()
        self.tello.streamon()

        frame_read = self.tello.get_frame_read()

        should_stop = False
        while not should_stop:

            for event in pygame.event.get():
                if event.type == pygame.USEREVENT + 1:
                    self.update()
                elif event.type == pygame.QUIT:
                    should_stop = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        should_stop = True
                    else:
                        self.keydown(event.key)
                elif event.type == pygame.KEYUP:
                    self.keyup(event.key)

            if frame_read.stopped:
                break

            self.screen.fill([0, 0, 0])

            frame = frame_read.frame

            # Find AR Markers
            try:
                corners, ids = aruco_detect.findArucoMarkers(frame, 6, 50)
                aruco_detect.centerloc(frame, corners, ids)
            except Exception as e:
                pass
                # print(f"No markers found!: {e}")

            self.a_id = -1
            if not (ids is None):
                self.a_id = ids[0]
            text = ""

            match self.a_id:
                case 0:
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
                    height, width = frame.shape[:2]
                    z_center = height / 2
                    z_center = int(z_center)
                    cv2.line(frame, (0, z_center), (width, z_center), (0, 255, 0), 10)
                    cv2.line(frame, (0, average_z_position), (width, average_z_position), (0, 0, 255), 10)

                    x_center = width / 2
                    x_center = int(x_center)

                    cv2.line(frame, (x_center, 0), (x_center, height), (0, 255, 0), 10)
                    cv2.line(frame, (average_x_position, 0), (average_x_position, height), (0, 0, 255), 10)
                    half_box_size = int(box_size / 2)
                    cv2.rectangle(frame, (x_center - half_box_size, z_center - half_box_size),
                                  (x_center + half_box_size, z_center + half_box_size), (255, 0, 0), 10)

                    # perform calculations
                    x_distance = average_x_position - x_center
                    y_distance = (area - box_size * box_size) / 100
                    z_distance = average_z_position - z_center
                    final_x_movement = x_pid(x_distance)
                    final_x_movement = -int(final_x_movement)
                    final_z_movement = z_pid(z_distance)
                    final_z_movement = int(final_z_movement)

                    final_y_movement = y_pid(y_distance)
                    final_y_movement = int(final_y_movement)
                    self.x = final_x_movement
                    self.y = final_y_movement
                    self.z = final_z_movement
                    self.yaw = 0
                case 1:
                    text = "State 1 detected. Bounce action initiated"
                case 2:
                    text = "State 2 detected. Sequence of flips initiated"
                case 3:
                    text = "State 3 detected. Landing the drone!"
                    land = True
                case _:
                    text = "No marker detected!"

            # place action on frame
            cv2.putText(frame, text, (10, 50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 3)

            # battery display
            text = "Tello Battery: {}%".format(self.tello.get_battery())
            cv2.putText(frame, text, (5, 710), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(frame, f"FPS: {FPS}", (7, 675), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = np.rot90(frame)
            frame = np.flipud(frame)

            frame = pygame.surfarray.make_surface(frame)
            self.screen.blit(frame, (0, 0))

            pygame.display.update()

            time.sleep(1 / FPS)

        # Deallocate Resources
        self.tello.end()

    def keydown(self, key):
        if key == pygame.K_c:  # set forward velocity
            self.for_back_velocity = S
        elif key == pygame.K_x:  # set backward velocity
            self.for_back_velocity = -S
        elif key == pygame.K_z:  # set left velocity
            self.left_right_velocity = -S
        elif key == pygame.K_v:  # set right velocity
            self.left_right_velocity = S
        elif key == pygame.K_w:  # set up velocity
            self.up_down_velocity = S
        elif key == pygame.K_s:  # set down velocity
            self.up_down_velocity = -S
        elif key == pygame.K_a:  # set yaw counter clockwise velocity
            self.yaw_velocity = -S
        elif key == pygame.K_d:  # set yaw clockwise velocity
            self.yaw_velocity = S

    def keyup(self, key):
        if key == pygame.K_c or key == pygame.K_v:  # set zero forward/backward velocity
            self.for_back_velocity = 0
        elif key == pygame.K_z or key == pygame.K_v:  # set zero left/right velocity
            self.left_right_velocity = 0
        elif key == pygame.K_w or key == pygame.K_s:  # set zero up/down velocity
            self.up_down_velocity = 0
        elif key == pygame.K_a or key == pygame.K_d:  # set zero yaw velocity
            self.yaw_velocity = 0
        elif key == pygame.K_t:  # takeoff
            self.tello.takeoff()
            self.send_rc_control = True
        elif key == pygame.K_l:  # land
            not self.tello.land()
            self.send_rc_control = False
        elif key == pygame.K_u:  # flip forward
            self.tello.flip('f')
        elif key == pygame.K_h:  # flip left
            self.tello.flip('l')
        elif key == pygame.K_j:  # flip backward
            self.tello.flip('b')
        elif key == pygame.K_k:  # flip right
            self.tello.flip('r')

    def update(self):
        """
        Asynchronous Update Routine
        """


        # Render States
        if self.send_rc_control:
            if self.a_id == 0:
                self.tello.send_rc_control(self.x, self.y, self.z, self.yaw)
                self.a_id = -1
            elif self.a_id == 1:
                self.tello.move_up(100)
                self.tello.flip('b')
                self.tello.move_down(100)
                self.a_id = -1
            elif self.a_id == 2:
                self.tello.flip('f')
                time.sleep(1)
                self.tello.flip('b')
                time.sleep(1)
                self.tello.flip('l')
                time.sleep(1)
                self.tello.flip('r')
                self.a_id = -1
            elif self.a_id == 3:
                self.a_id = -1
                not self.tello.land()
                self.send_rc_control = False
            # Change Tello velocities
            else:
                x_pid.auto_mode = False
                y_pid.auto_mode = False
                z_pid.auto_mode = False

                self.tello.send_rc_control(self.left_right_velocity, self.for_back_velocity,
                                           self.up_down_velocity, self.yaw_velocity)
        else:
            if self.a_id == 3:
                self.a_id = -1
                self.tello.takeoff()
                self.send_rc_control = True


def main():
    run_tello = TelloUI()
    run_tello.run()


if __name__ == '__main__':
    main()