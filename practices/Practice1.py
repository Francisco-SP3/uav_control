# Francisco Salas Porras A01177893 UAVs 24/04/2024

from djitellopy import Tello
import time

tello = Tello()

tello.connect()

time.sleep(2)

tello.takeoff()

time.sleep(2)

tello.move_up(100)

time.sleep(3)

tello.land()