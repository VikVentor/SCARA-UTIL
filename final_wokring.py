import cv2
import numpy as np
import serial
import time

# Define the lower and upper bounds for the brown color
lower_brown = np.array([20, 50, 50])
upper_brown = np.array([80, 255, 255])

# Set minimum area threshold
min_area_threshold = 0.1  # Adjust as needed

# Initialize the webcam
cap = cv2.VideoCapture(4)

# Initialize the Arduino connection
ser = serial.Serial('/dev/ttyACM0', 9600)  # Change 'COM3' to your Arduino port

# Main loop
while True:
    ret, frame = cap.read()

    # Convert the frame to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask for the brown color
    mask = cv2.inRange(hsv, lower_brown, upper_brown)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Process each contour
    max_area = 0
    selected_contour = None

    for contour in contours:
        # Calculate the bounding box for each contour
        x, y, w, h = cv2.boundingRect(contour)
        
        # Calculate the area of the bounding box
        area = w * h

        # Select the contour with the maximum area and above the threshold
        if area > max_area and area >= min_area_threshold * (frame.shape[0] * frame.shape[1]):
            max_area = area
            selected_contour = contour
    

    
    # Draw a bounding box around the selected contour
    if selected_contour is not None:
        x, y, w, h = cv2.boundingRect(selected_contour)
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

        # Check the received character from Arduino
    if ser.in_waiting > 0:
        received_char = ser.read().decode('utf-8')
        if received_char == 'A':
            if selected_contour is not None:
                print("received A")
                ser.write(b'1')
            elif selected_contour is None:
                ser.write(b'3')
        elif received_char == 'B':
            ser.write(b'4')
                
        

    # Display the output frame
    cv2.imshow('Frame', frame)

    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the resources
cap.release()
cv2.destroyAllWindows()
