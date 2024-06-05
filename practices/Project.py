from djitellopy import Tello
import math
import time
import cv2
from threading import Thread


tello = Tello()

print("Connecting to Tello")
tello.connect()

print("Starting video stream")
tello.streamon()
frame_reader = tello.get_frame_read()
keepRecording = True

def thread_func():
    # Main loop to display the video stream
    try:
        while keepRecording:
            frame = frame_reader.frame
            filter = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            cv2.imshow("Tello Camera Filtered", filter)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
    finally:
        tello.streamoff()
        cv2.destroyAllWindows()

recorder = Thread(target=thread_func)
recorder.start()

print("Taking off")
tello.takeoff()

period = 30
time_delta = period / 360

print("Starting movement")
#tello.send_rc_control(20, 0, 0, 0)
#time.sleep(3)
#print("Starting turn")
#tello.send_rc_control(0, 0, 0, 20)
#time.sleep(3)
#for i in range(360):
#    tello.send_rc_control(-20, 0, 0, 20)
#    time.sleep(time_delta)

tello.send_rc_control(-20, 0, 0, 50)
print("Ended movement")
time.sleep(10)

print("Stopping")
tello.send_rc_control(0, 0, 0, 0)
time.sleep(3)

keepRecording = False
recorder.join()

print("Landing")
tello.land()
