from djitellopy import tello
from time import sleep

drone = tello.Tello()
drone.connect()
print(drone.get_battery())

print('Taking off')
drone.takeoff()
print('Move right with speed 30')
# drone.send_rc_control(0, 0, 0, 50)
# sleep(1)
# drone.send_rc_control(0, 0, 20, 0)
# sleep(2)
print('Move forward with speed 50')
drone.send_rc_control(0, 50, 0, 0)  # speed
sleep(2)
print('Hover then land')
drone.send_rc_control(0, 0, 0, 0)
drone.land()
