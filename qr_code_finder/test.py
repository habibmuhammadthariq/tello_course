import cv2
from djitellopy import tello
from tello_course.qr_code_finder import simple_find_contours as reader

drone = tello.Tello()
drone.connect()
print(f'Battery : {drone.get_battery()}')

drone.streamon()
frame_read = drone.get_frame_read()

while True:
    frame = frame_read.frame
    frame = cv2.resize(frame, (360,240))
    # frame = cv2.flip(frame, 1)

    frame, status = reader.extract(frame, True)
    if status:
        direction = reader.get_direction()
        print("Next destination : {}".format(direction))

    # show the image up
    cv2.imshow("frame", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("I quit!")
        frame_read.stop()
        drone.streamoff()
        break


