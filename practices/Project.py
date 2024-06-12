from djitellopy import Tello
import math, sys, time, cv2
from threading import Thread
from Circle import Circle
from enum import IntEnum

# Parameters
circleRadius = 900
circleHeight = 70
k_x = -0.28
k_y = -0.06
k_z = 0.24
kd_x = 0.0
kd_y = 0.0
kd_z = 0.0

# Variables
firstFrame = True
keepRunning = True
moveable = True
clockwise = True

# Pose 
x = 480
y = circleRadius
z = 360

# Deseada
x_d = 0
y_d = 0
z_d = 0
if clockwise == True: turn = -10
else: turn = 10

# State
class States(IntEnum):
    FIND_CIRCLE = 0
    GOTO_CIRCLE = 1
    ROTATE_CIRCLE = 2

# Initialize the Tello object
tello = Tello()
print("Connecting to Tello")
tello.connect()

print("Starting video stream")
tello.streamon()
frame_reader = tello.get_frame_read()

# Circle
ball = Circle(100)

# Capture video and detect circle
def camera_processing():
    # Main loop to display the video stream
    try:
        global firstFrame, keepRunning, x_d, y_d, z_d
        while keepRunning:
            frame = frame_reader.frame
            frame_RGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame, frame_treshold, center, distance = ball.circleDetection(frame, frame_RGB)
            x_d = center[0]
            y_d = distance
            z_d = center[1]
            cv2.imshow("Tello Camera Filtered", frame)
            cv2.imshow("Tello Camera Treshold", frame_treshold)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                keepRunning = False
                break
    finally:
        tello.streamoff()
        cv2.destroyAllWindows()

# Drone movement control
def controller(moveFlag):
    try:
        global keepRunning, x, y, z, x_d, y_d, z_d, turn
        while keepRunning:        
            # Calculate the error
            e_x = x - x_d
            e_y = y - y_d
            e_z = z - z_d

            # Calculate the control signal
            u_x = k_x * e_x + kd_x
            u_y = k_y * e_y + kd_y
            u_z = k_z * e_z + kd_z

            prt = str(u_x) + ", " + str(u_y) + ", " + ", " + str(u_z)
            print(prt)
            # Send the control signal to the drone
            if y_d != 0 and moveFlag:
                if u_x < 10 and u_y < 10 and u_z < 10:
                    tello.send_rc_control(turn, int(u_y), int(u_z), int(u_x))
                else:
                    tello.send_rc_control(0, int(u_y), int(u_z), int(u_x))
            else:
                tello.send_rc_control(0, 0, 0, 20)

            
    finally:
        tello.send_rc_control(0, 0, 0, 0)
        print("Landing")
        tello.land()
        

#print("Starting movement")
#tello.send_rc_control(-20, 0, 0, 50)
#print("Ended movement")
#time.sleep(10)

def main(argv):
    recorder = Thread(target=camera_processing)
    recorder.start()

    print("Taking off")
    tello.takeoff()
    tello.move_up(circleHeight)

    time.sleep(2)

    print("Starting movement")
    control = Thread(target=controller(moveable))
    control.start()
    #controller(moveable)

    if not keepRunning:
        print("Stopping")
        control.join()
        recorder.join()

    return 0

if __name__ == "__main__":
 main(sys.argv[1:])
