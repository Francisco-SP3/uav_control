# Take 10 photos of the chessboard and save them
from djitellopy import Tello
import sys, os, cv2

# Path to the folder where the photos will be saved
path = "practices/chessboard_drone"

# Create the folder if it does not exist
if not os.path.exists(path):
    os.makedirs(path)

# Connect to the drone
tello = Tello()
print("Connecting to Tello")
tello.connect()

# Start the video stream
print("Starting video stream")
tello.streamon()

def main(argv):
 # Initialize the webcam
 frame_reader = tello.get_frame_read()
 i = 0

 while True:
  frame = frame_reader.frame

  cv2.imshow("Press 'c' to take a photo", frame)

  key = cv2.waitKey(30)
  if key == ord('c') or key == 99:
   print("Photo " + str(i+1) + " taken.")
   cv2.imwrite(path + "/photo" + str(i) + ".jpg", frame)
   i += 1
  if key == ord('q') or key == 27:
   break
  if i == 10:
   break

if __name__ == "__main__":
 main(sys.argv[1:])