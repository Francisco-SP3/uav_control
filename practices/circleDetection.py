from djitellopy import Tello
import numpy as np
import sys, cv2
from Circle import Circle

# Connect to the drone
tello = Tello()
print("Connecting to Tello")
tello.connect()

# Start the video stream
print("Starting video stream")
tello.streamon()

def main(argv):
 
 # Start capturing the video
 frame_reader = tello.get_frame_read()

 # Circle
 ball = Circle(100)

 while True:
  # Capture frame-by-frame
  frame = frame_reader.frame
  if frame is None:
   print("Error: no frame.")
   break

  # Apply the circle detection
  frame_RGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
  #frame, frame_treshold, center, distance = circleDetection(frame, frame_RGB)
  frame, frame_treshold, center, distance = ball.circleDetection(frame, frame_RGB)

  # Display the resulting frame
  cv2.imshow('Circle Detection', frame_RGB)
  cv2.imshow('Circle Detection Treshold', frame_treshold)

  key = cv2.waitKey(30)
  if key == ord('q') or key == 27:
   break
 return 0

if __name__ == "__main__":
 main(sys.argv[1:])
  