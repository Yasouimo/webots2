import csv
import numpy as np
import cv2 as cv
from controller import Robot
from ultralytics import YOLO
import time

# Initialize the robot
robot = Robot()

# Time step for the simulation
timeStep = int(robot.getBasicTimeStep())

# Constants for violation logging
csv_file = "violations.csv"
header = ['Object', 'Violation Type', 'Timestamp']

# Create CSV file for logging violations
with open(csv_file, mode='w') as file:
    writer = csv.writer(file)
    writer.writerow(header)

# Initialize YOLO model for object detection
model = YOLO("models/best.pt")  # Assuming the path to the YOLO weights file

# Camera setup (updated)
camera = robot.getDevice("camera")  # Use getDevice instead of getCamera
camera.enable(timeStep)

# Log violations in CSV
def log_violation(object_name, violation_type):
    timestamp = robot.getTime()
    with open(csv_file, mode='a') as file:
        writer = csv.writer(file)
        writer.writerow([object_name, violation_type, timestamp])

# Alarm function (example)
def trigger_alarm():
    print("ALARM! Cat or Rabbit detected!")

# Function to detect objects using YOLO
def detect_objects(camera):
    # Capture the current image from the camera (RGBA format)
    image = camera.getImage()
    
    # Convert the image from RGBA to BGR (Webots camera output is RGBA)
    image_array = np.frombuffer(image, dtype=np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))  # RGBA
    image_bgr = cv.cvtColor(image_array, cv.COLOR_RGBA2BGR)  # Convert RGBA to BGR

    # Perform inference using YOLO model
    results = model(image_bgr)  # Pass the BGR image to the YOLO model
    
    detected_objects = []
    
    # Process the results
    for result in results:
        for obj in result.boxes:
            class_id = int(obj.cls)
            confidence = obj.conf
            if confidence > 0.5:  # Only consider detections with high confidence
                object_name = model.names[class_id]
                detected_objects.append({'name': object_name, 'confidence': confidence})
    
    return detected_objects

while robot.step(timeStep) != -1:
    # Detect objects using YOLO
    detected_objects = detect_objects(camera)
    
    for obj in detected_objects:
        object_name = obj['name']
        
        # If the object is a cat or rabbit, trigger alarm and log the violation
        if object_name == 'cat' or object_name == 'rabbit':
            trigger_alarm()
            log_violation(object_name, "red")  # Log violation with a "red" alert
        else:
            log_violation(object_name, "orange")  # Log violation with an "orange" alert
