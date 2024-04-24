import cv2
import numpy as np

import serial
import time

# Serial communication setup
ser = serial.Serial('/dev/ttyUSB0', 9600)  # Change '/dev/ttyACM0' to the correct port
time.sleep(2)  # Allow time for Arduino to initialize

# Function to send data to Arduino
def send_to_arduino(data):
    ser.write(data)

# Function to calculate distance from stereo camera disparity
def calculate_distance(disparity):
    # Focal length 
    focal_length = 3.04  # 

    # Baseline distance between the two cameras (in centimeters)
    baseline = 6.0  

    # Computing distance 
    distance = (focal_length * baseline * 100) / (disparity + 0.0001)
    return distance

# Main function to detect obstacles and control robot
def detect_obstacles():
    # Initializing stereo camera (Left and Right camera)
    left_camera = cv2.VideoCapture(0)
    right_camera = cv2.VideoCapture(3)

    while True:
        # Capture frames from left and right cameras
        ret1, left_frame = left_camera.read()
        ret2, right_frame = right_camera.read()

        if not ret1 or not ret2:
            print("Failed to capture frames")
            break

        
        width = 640  
        height = 480  
        left_frame = cv2.resize(left_frame, (width, height))
        right_frame = cv2.resize(right_frame, (width, height))

        # Converting frames to grayscale
        left_gray = cv2.cvtColor(left_frame, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_frame, cv2.COLOR_BGR2GRAY)

        # Performing stereo matching to compute disparity map
        stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
        disparity = stereo.compute(left_gray, right_gray)

        # Normalize disparity map
        normalized_disparity = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

        # Find contours in the normalized disparity map
        contours, _ = cv2.findContours(normalized_disparity, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Loop through detected contours
        for contour in contours:
            # Calculating area of contour
            area = cv2.contourArea(contour)

            # If area is above a threshold, consider it as an obstacle
            if area > 100:
                
                distance = calculate_distance(area)
                print("Distance to obstacle:", distance, "centimeters")
                

                # If distance is less than 10cm, stop the robot
                if distance < 10:
                    print("Obstacle detected! Stopping the robot.")
                    send_to_arduino(b'1')
                    

                # Draw distance information on the frame
                cv2.putText(right_frame, f"Distance: {distance:.10f} cm", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1, cv2.LINE_AA)

        # Display frames
        cv2.imshow('Left Frame', left_frame)
        cv2.imshow('Right Frame', right_frame)
        cv2.imshow('Normalized Disparity', normalized_disparity)
        
        
        
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    
    left_camera.release()
    right_camera.release()
    cv2.destroyAllWindows()


detect_obstacles()
