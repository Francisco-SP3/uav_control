# Francisco Salas Porras A01177893 UAVs 24/04/2024

from djitellopy import Tello
import time

tello = Tello()

tello.connect()

tello.takeoff()

tello.move_forward(100)

tello.rotate_clockwise(90)

tello.move_forward(100)

tello.rotate_clockwise(90)

tello.move_forward(100)

tello.rotate_clockwise(90)

tello.move_forward(100)

tello.rotate_clockwise(90)

time.sleep(3)

tello.land()