import time
from djitellopy import tello
import cv2

drone = tello.Tello()
drone.connect()
print(drone.get_battery())

drone.streamon()
frame_read = drone.get_frame_read()

while True:
    img = frame_read.frame
    # resize the image in order to decrease the computation
    # img = cv2.resize(img, (360, 240))

    cv2.imshow('Camera show', img)

    # take a picture from the drone
    # key = input('Input value : ')
    # if key == 'c':
    #     cv2.imwrite(f'img/{time.time()}.jpg', img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# stop getting image from the drone
frame_read.stop()
# turn off the camera
drone.streamoff()