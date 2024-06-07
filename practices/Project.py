from djitellopy import Tello
import math
import time
import cv2
from threading import Thread
import circleDetection

# Parameters

# Variables
firstFrame = True

# Desired 
a

# Initialize the Tello object
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
            if firstFrame:
                (h, w, _) = frame.shape
                im_center = (w//2, h//2)
                firstFrame = False
            frame_RGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame, frame_treshold, center, distance = circleDetection.circleDetection(frame, frame_RGB)
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



print("Starting movement")
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
