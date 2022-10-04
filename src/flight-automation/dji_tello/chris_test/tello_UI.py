from djitellopy import Tello
import cv2
import pygame
import numpy as np
import time
import aruco_detect

# Speed of the drone
S = 60
# Frames per second of the pygame window display
FPS = 120


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
        if key == pygame.K_UP:  # set forward velocity
            self.for_back_velocity = S
        elif key == pygame.K_DOWN:  # set backward velocity
            self.for_back_velocity = -S
        elif key == pygame.K_LEFT:  # set left velocity
            self.left_right_velocity = -S
        elif key == pygame.K_RIGHT:  # set right velocity
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
        if key == pygame.K_UP or key == pygame.K_DOWN:  # set zero forward/backward velocity
            self.for_back_velocity = 0
        elif key == pygame.K_LEFT or key == pygame.K_RIGHT:  # set zero left/right velocity
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
                self.tello.rotate_clockwise(360)
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