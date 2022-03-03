from djitellopy import tello
import keyPressModule as kp
import time
import cv2

# initiate keyPressedModule
kp.init()

# declare image variable globally
global img

# initiate tello drone class
drone = tello.Tello()
drone.connect()
print(drone.get_battery())

drone.streamon()
frame_read = drone.get_frame_read()

def getKeyboardInput():
    lf, fb, ud, yv = 0, 0, 0, 0
    speed = 50

    if kp.getKey("LEFT"):
        lf = -speed
    elif kp.getKey("RIGHT"):
        lf = speed

    if kp.getKey("UP"):
        fb = speed
    elif kp.getKey("DOWN"):
        fb = -speed

    if kp.getKey("w"):
        ud = speed
    elif kp.getKey("s"):
        ud = -speed

    if kp.getKey("a"):
        yv = -speed
    elif kp.getKey("d"):
        yv = speed

    if kp.getKey("q"): drone.land(); time.sleep(3)
    if kp.getKey("e"): drone.takeoff()

    if kp.getKey("z"):
        cv2.imwrite(f'img/{time.time()}.jpg', img)
        time.sleep(0.3)  # this is need to stop the looping.
        # if we doesn't have one, the image capture will be so much

    return [lf, fb, ud, yv]


while True:
    vals = getKeyboardInput()
    drone.send_rc_control(vals[0], vals[1], vals[2], vals[3])
    # time.sleep(0.05)

    img = frame_read.frame
    img = cv2.resize(img, (360, 240))

    cv2.imshow('Image', img)
    cv2.waitKey(1)

