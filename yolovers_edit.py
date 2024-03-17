import cv2
import serial
from ultralytics import YOLO

# Set minimum area threshold
min_area_threshold = 0.1  # Adjust as needed

# Initialize the webcam
cap = cv2.VideoCapture(4)  # Change the camera index as needed

# Initialize the Arduino connection
ser = serial.Serial('/dev/ttyUSB0', 115200)  # Change 'COM3' to your Arduino port

# Initialize YOLO
yolo = YOLO("use.pt")  # You can use other YOLO variants like "yolov5m", "yolov5l", "yolov5x" based on your requirements

# Main loop
while True:
    ret, frame = cap.read()

    # Detect objects using YOLO
    results = yolo(frame, conf = 0.8)

    # Process YOLO results
    for det in results[0].boxes.xyxy:
        x, y, w, h = map(int, det[:4])
        area = w * h

        # Select the bounding box with an area above the threshold
        if area >= min_area_threshold * (frame.shape[0] * frame.shape[1]):
            # Draw bounding box
            cv2.rectangle(frame, (x, y), (w, h), (0, 255, 0), 2)

            # Check the received character from Arduino
    if ser.in_waiting > 0:
        received_char = ser.read().decode('utf-8')
        print(received_char)
        if det in results:
            if received_char == 'A':
                print(received_char)
                ser.write(b'1')
            elif received_char == 'B':
                ser.write(b'4')
        else:
            ser.write(b'0')

    annotated_frame = results[0].plot(conf = True)

    # Display the output frame
    cv2.imshow('Frame', annotated_frame)

    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the resources
cap.release()
cv2.destroyAllWindows()
