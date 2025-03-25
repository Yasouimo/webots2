from controller import Robot, Camera, Motor

TIME_STEP = 50  # Ensure it matches Webots' world step

# Initialize the robot
robot = Robot()

# Initialize Motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float('inf'))  # Set to velocity control
right_motor.setPosition(float('inf'))

left_motor.setVelocity(0.0)  # Initial speed
right_motor.setVelocity(0.0)

# Initialize Camera
camera = robot.getDevice("camera")
camera.enable(TIME_STEP)  # Enable camera

# Main Control Loop
while robot.step(TIME_STEP) != -1:
    # Capture Camera Image
    image = camera.getImage()
    if image:
        print("Camera capturing frames.")

    # Move the robot forward
    left_motor.setVelocity(3.0)
    right_motor.setVelocity(3.0)  # Adjust speed if needed

