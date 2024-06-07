# Define Circle class
import cv2
import numpy as np

class Circle:
    def __init__(self, ballD):
        # Parameters
        self.ballD = ballD # Ball diameter [mm]
        self.hsv_parameters = [25, 45, 90, 180, 40, 255] # Color filter parameters
        self.hough_params = [50, 10, 15, 150] # Hough Circle Transform parameters [param1, param2, minRadius, maxRadius]
        self.erodeI = 1 # Erode iterations
        self.dilateI = 1 # Dilate iterations
        self.blurI = 3 # Blur iterations
        self.max_lenC = 5 # Center memory length for Mean Filter
        self.max_lenR = 10 # Radius memory length for Mean Filter
        self.camera_matrix = [[ 908.2856024835826 ,  0.0 ,  482.87858112836204 ], [ 0.0 ,  901.8162522058758 ,  333.0625329073474 ], [ 0.0 ,  0.0 ,  1.0 ]]
        
        # Mean Filter variables
        self.center_memory = [None] * self.max_lenC
        self.radius_memory = [None] * self.max_lenR
        self.lenC = 0
        self.lenR = 0
        self.j = 0
        self.k = 0

        # Camera variables
        self.im_center = (480, 360)
        self.f_x = self.camera_matrix[0][0] # Focal length in x
        self.f_y = self.camera_matrix[1][1] # Focal length in y
        self.f = (self.f_x + self.f_y) / 2 # Focal length
        self.numD = self.ballD * self.f # Numerator of the distance calculation


    def filter(self, center, radius):
        resultC = (0, 0)
        resultR = 0
        acumC1 = 0
        acumC2 = 0
        acumR = 0

        self.center_memory[self.j] = center
        self.radius_memory[self.k] = radius

        if self.lenC < self.max_lenC:
            self.lenC = self.lenC + 1

        if self.lenR < self.max_lenR:
            self.lenR = self.lenR + 1

        for i in range(self.lenC):
            acumC1 += self.center_memory[i][0]
            acumC2 += self.center_memory[i][1]
        resultC = (int(acumC1 / self.lenC), int(acumC2 / self.lenC))
        self.j = self.j + 1

        for i in range(self.lenR):
            acumR += self.radius_memory[i]
        resultR = int(acumR / self.lenR)
        self.k = self.k + 1

        if self.j >= self.max_lenC:
            self.j = 0

        if self.k >= self.max_lenR:
            self.k = 0

        return resultC, resultR

    def circleDetection(self, frame, frame_RGB):
        filter_center = (0, 0)
        distance = 0

        frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame_threshold = cv2.inRange(frame_HSV, (self.hsv_parameters[0], self.hsv_parameters[2], self.hsv_parameters[4]), (self.hsv_parameters[1], self.hsv_parameters[3], self.hsv_parameters[5]))
        frame_erode = cv2.erode(frame_threshold, None, iterations=self.erodeI)
        frame_dilate = cv2.dilate(frame_erode, None, iterations=self.dilateI)
        frame_blur = cv2.medianBlur(frame_dilate, self.blurI)

        # Apply the Hough Circle Transform
        rows = frame_blur.shape[0]
        circles = cv2.HoughCircles(frame_blur, cv2.HOUGH_GRADIENT, 1, rows / 8,
        param1=self.hough_params[0], param2=self.hough_params[1],
        minRadius=self.hough_params[2], maxRadius=self.hough_params[3])
        # Draw the circles
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                # Circle center and radius
                center = (i[0], i[1])
                radius = i[2]
                # Filter the center and radius
                filter_center, filter_radius = filter(center, radius)
                # Draw the circle
                cv2.circle(frame_RGB, filter_center, 1, (0, 100, 100), 3)
                cv2.circle(frame_RGB, filter_center, filter_radius, (255, 0, 255), 3)
            # Draw distance from circle to center
            frame_RGB = self.center2circle(self, frame_RGB, center)
            # Calculate distance from camera to circle
            frame_RGB, distance = self.camera2circle(self, frame_RGB, radius)

        return frame_RGB, frame_blur, filter_center, distance
    
    def center2circle(self, frame, center):
        if(center != (0, 0)):
            # Draw the center and the distance
            cv2.circle(frame, self.im_center, 1, (0, 100, 100), 3)
            cv2.arrowedLine(frame, center, self.im_center, (0, 0, 255), 3)
        return frame
    
    def camera2circle(self, frame, radius):
        if(radius != 0):
            # Calculate the camera distance
            distance = self.numD / (2*radius) # Distance in [mm]
            # Print the distance
            cv2.putText(frame, "Distance: " + str(distance) + " mm", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        else:
            distance = 0
        return frame, distance