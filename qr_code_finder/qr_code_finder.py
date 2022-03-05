import math
import cv2
import numpy as np
from tello_course.qr_code_finder import simple_find_contours as qr_finder
from djitellopy import tello
import time

# cap = cv2.VideoCapture(0)     # 1

drone = tello.Tello()
drone.connect()
print(f'Battery : {drone.get_battery()}')

# start stream
drone.streamon()
frame_read = drone.get_frame_read()

pid = [0.4, 0.4, 0]
pError, pDistance = 0, 0

# print('Taking off')                          # b - 3 rows below     -> '
# drone.takeoff()                              #
# time.sleep(1)
# drone.send_rc_control(0, 0, 20, 0)
# time.sleep(2)


def trackQrCode(direction, distance, error):
    fb, ud = 0, False

    speed = pid[0] * error + pid[1] * (error - pError)
    speed = int(np.clip(speed, -100, 100))

    if direction == 'up' or direction == 'down':
        ud = True
    elif direction == 'hover':
        fb = 0
    elif direction == 'forward':
        fb = 20
    elif direction == 'backward':
        fb = -20

    print(fb, speed)

    # send_rc_control was command based on velocity. so it's cm/s.
    # the time determine by the sleep. time.sleep(1) mean 1 seconds.
    # example : 20 -> 20 cm in 1 seconds.
    # So the drone will move 20 cm on 1 seconds
    if pDistance != 0 and (math.fabs(distance - pDistance) > 30):
        print('Detection failed -> Hover')  # temporary
    #     drone.send_rc_control(0, 0, 0, 0)               # d - 6 rows below    -> '
    # elif ud:
    #     drone.send_rc_control(0, 0, speed, 0)
    # elif fb != 0:
    #     drone.send_rc_control(0, fb, 0, speed)
    # else:
    #     drone.send_rc_control(0, 0, 0, 0)


while True:
    # _, img = cap.read()               # 2
    img = frame_read.frame                              # e
    img = cv2.resize(img, (360, 240))

    # image with detected qr code, the qr_code detected or not
    frame, status = qr_finder.extract(img, True)

    if status:
        # getting error between qr center with frame center
        error = qr_finder.get_error()
        # direction for the drone to come into qr code.
        direction, distance = qr_finder.get_direction()
        print(direction)  # temporary
        # send command to the drone
        trackQrCode(direction, distance, error)
        # update pError
        pError = error
        pDistance = distance
    else:
        # drone.send_rc_control(0, 0, 0, 0)               # f         -> '
        pass                                    # 3

    cv2.imshow('Output', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        # cv2.imwrite("../img/qr_code.jpg", img)
        # drone.send_rc_control(0, 0, 0, 0)               # g - 2 row below   -> '
        # time.sleep(0.2)
        # drone.land()
        drone.streamoff()
        break