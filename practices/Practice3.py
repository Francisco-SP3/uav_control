# Francisco Salas Porras A01177893 UAVs 15/05/2024

from djitellopy import Tello
import time
import math
import cv2



def make_circle(drone_controller, radius=1, duration=10):
    # Set the desired velocity components to maintain circular motion
    angular_speed = 360 / duration  # degrees per second
    time_interval = 0.1  # seconds

    start_time = time.time()
    angle = 0
    while angle < 360:
        elapsed_time = time.time() - start_time
        angle = angular_speed * elapsed_time  # degrees
        x = radius * math.cos(math.radians(angle))
        y = radius * math.sin(math.radians(angle))
        
        # Adjust velocities to maintain circular motion in the XY plane
        forward_backward_velocity = int(x) * 10  # Scale to match the input range (-100~100)
        left_right_velocity = int(y) * 10  # Scale to match the input range (-100~100)
        up_down_velocity = 0  # No forward/backward movement
        yaw_velocity = 100  # Rotate continuously to maintain circular trajectory

        # Send control commands to the drone
        drone_controller.send_rc_control(left_right_velocity, forward_backward_velocity, up_down_velocity,
                                         yaw_velocity)

        time.sleep(time_interval)

def make_circle_pointing_inwards(drone_controller, radius=1, duration=10):
    # Set the desired velocity components to maintain circular motion
    angular_speed = 360 / duration  # degrees per second
    time_interval = 0.1  # seconds

    start_time = time.time()
    angle = 0
    while angle < 360:
        elapsed_time = time.time() - start_time
        angle = angular_speed * elapsed_time  # degrees
        x = radius * math.cos(math.radians(angle))
        y = radius * math.sin(math.radians(angle))
        
        # Adjust velocities to maintain circular motion in the XY plane
        forward_backward_velocity = int(x) * 100  # Scale to match the input range (-100~100)
        left_right_velocity = int(y) * 100  # Scale to match the input range (-100~100)
        up_down_velocity = 0  # No forward/backward movement
        yaw_velocity = -100  # Rotate continuously to maintain circular trajectory

        # Send control commands to the drone
        drone_controller.send_rc_control(left_right_velocity, forward_backward_velocity, up_down_velocity,
                                         yaw_velocity)

        time.sleep(time_interval)


def stop_drone(drone_controller):
    drone_controller.send_rc_control(0, 0, 0, 0)

tello = Tello()

tello.connect()

tello.takeoff()

make_circle(tello, radius=10, duration=50)

stop_drone(tello)

tello.land()



#tello.streamon()

#time.sleep(1)

#backgroundFrameRead = tello.get_frame_read()

#cv2.imshow("frame", backgroundFrameRead.frame)