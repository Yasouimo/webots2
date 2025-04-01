from controller import Robot

# Initialize the robot
robot = Robot()

# Time step for the simulation
timeStep = int(robot.getBasicTimeStep())

# Constants for the e-puck motors and distance sensors
maxMotorVelocity = 4.2
num_left_dist_sensors = 4
num_right_dist_sensors = 4
left_threshold = 75  # Threshold for the left sensors
right_threshold = 75  # Threshold for the right sensors

# Get motors
leftMotor = robot.getMotor("left wheel motor")
rightMotor = robot.getMotor("right wheel motor")

# Get distance sensors for the left and right side
dist_left_sensors = [robot.getDistanceSensor('ps' + str(x)) for x in range(num_left_dist_sensors)]
dist_right_sensors = [robot.getDistanceSensor('ps' + str(x + num_left_dist_sensors)) for x in range(num_right_dist_sensors)]

# Enable the distance sensors
for sensor in dist_left_sensors + dist_right_sensors:
    sensor.enable(timeStep)

# Set the motor position to infinity for velocity control
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Set the initial velocity of the motors
initialVelocity = 0.7 * maxMotorVelocity
leftMotor.setVelocity(initialVelocity)
rightMotor.setVelocity(initialVelocity)

while robot.step(timeStep) != -1:
    # Read sensor values
    left_dist_sensor_values = [sensor.getValue() for sensor in dist_left_sensors]
    right_dist_sensor_values = [sensor.getValue() for sensor in dist_right_sensors]
    
    # Check if there are obstacles in the left or right sensors
    left_obstacle = any(x > left_threshold for x in left_dist_sensor_values)
    right_obstacle = any(x > right_threshold for x in right_dist_sensor_values)
    
    # If there's an obstacle on the left, turn to the right
    if left_obstacle:
        leftMotor.setVelocity(initialVelocity - (0.5 * initialVelocity))  # Slow down left motor
        rightMotor.setVelocity(initialVelocity + (0.5 * initialVelocity))  # Speed up right motor
    
    # If there's an obstacle on the right, turn to the left
    elif right_obstacle:
        leftMotor.setVelocity(initialVelocity + (0.5 * initialVelocity))  # Speed up left motor
        rightMotor.setVelocity(initialVelocity - (0.5 * initialVelocity))  # Slow down right motor
    
    # If there are no obstacles, continue forward
    else:
        leftMotor.setVelocity(initialVelocity)
        rightMotor.setVelocity(initialVelocity)
