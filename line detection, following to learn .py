import cv2
import numpy as np
import serial
import time

# Serial communication setup
ser = serial.Serial('/dev/ttyUSB0', 9600)  # Change '/dev/ttyACM0' to the correct port
time.sleep(2)  # Allow time for Arduino to initialize

# send data to Arduino
def send_to_arduino(data):
    ser.write(data)

# Capture video from the camera 
cap = cv2.VideoCapture(0)

# Function to detect lines
def detect_line(frame):
    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Applying Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Detecting edges using Canny edge detector
    edges = cv2.Canny(blurred, 50, 150)
    
    # Perform Hough line transform to detect lines
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=100, maxLineGap=10)
    
    if lines is not None:
        return True
    else:
        return False

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Detect line
    line_detected = detect_line(frame)

    # If line is detected, send 1 to Arduino
    if line_detected:
        send_to_arduino(b'1')

    # Display the resulting frame
    cv2.imshow('Line Detection', frame)

    # Exit loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture
cap.release()
cv2.destroyAllWindows()
