from controller import Robot
import random
import math

# Initialize the robot
robot = Robot()

# Time step for the simulation
time_step = int(robot.getBasicTimeStep())

# Max motor speed (e-puck max speed is ~6.28 rad/s)
max_speed = 6.28

# Get motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

# Enable motors for velocity control
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Get distance sensors (8 sensors on e-puck)
num_sensors = 8
sensors = [robot.getDevice(f'ps{i}') for i in range(num_sensors)]
for sensor in sensors:
    sensor.enable(time_step)

# Speed settings
forward_speed = 0.5 * max_speed
turn_speed = 0.8 * max_speed  # Faster turning for escaping obstacles
reverse_speed = -0.3 * max_speed  # Reverse speed for collision recovery

# Sensor thresholds (tuned to avoid walls & ignore floor tiles)
OBSTACLE_THRESHOLD = 80  # Values above this indicate an obstacle
WALL_THRESHOLD = 150     # Very close to a wall

# Q-learning parameters
alpha = 0.3  # Learning rate
gamma = 0.9  # Discount factor
epsilon = 0.4  # Exploration rate
epsilon_decay = 0.995  # Decay per step
min_epsilon = 0.1  # Minimum exploration

# Actions: Forward, Sharp Left, Sharp Right, Slight Left, Slight Right
actions = ["Forward", "Sharp Left", "Sharp Right", "Slight Left", "Slight Right"]

# Q-table: state -> action values
Q_table = {}

# Track visited states for exploration bonus
visited_states = {}

# Function to discretize sensor readings into a state
def get_state(sensor_values):
    # Group sensors: front (0,7), left (5,6), right (1,2)
    front = max(sensor_values[0], sensor_values[7])
    left = max(sensor_values[5], sensor_values[6])
    right = max(sensor_values[1], sensor_values[2])
    
    # Discretize into 3 levels: 0 (clear), 1 (near), 2 (collision)
    state = []
    for value in [front, left, right]:
        if value < OBSTACLE_THRESHOLD:
            state.append(0)
        elif value < WALL_THRESHOLD:
            state.append(1)
        else:
            state.append(2)
    return tuple(state)

# Choose action using epsilon-greedy policy
def choose_action(state):
    global epsilon
    epsilon = max(min_epsilon, epsilon * epsilon_decay)
    
    if state not in Q_table:
        Q_table[state] = {a: 0 for a in actions}
    
    if random.random() < epsilon:
        return random.choice(actions)  # Explore
    else:
        return max(Q_table[state].items(), key=lambda x: x[1])[0]  # Exploit

# Update Q-table with exploration bonus
def update_Q(state, action, reward, next_state):
    if state not in Q_table:
        Q_table[state] = {a: 0 for a in actions}
    if next_state not in Q_table:
        Q_table[next_state] = {a: 0 for a in actions}
    
    # Exploration bonus (inverse of visit count)
    visited_states[state] = visited_states.get(state, 0) + 1
    exploration_bonus = 1.0 / math.sqrt(visited_states[state])
    
    # Q-learning update
    best_next_q = max(Q_table[next_state].values())
    Q_table[state][action] += alpha * (reward + exploration_bonus + gamma * best_next_q - Q_table[state][action])

# Reward function (heavily penalizes collisions)
def get_reward(sensor_values, action):
    front = max(sensor_values[0], sensor_values[7])
    left = max(sensor_values[5], sensor_values[6])
    right = max(sensor_values[1], sensor_values[2])
    
    # Heavy penalty for collisions
    if front > WALL_THRESHOLD or left > WALL_THRESHOLD or right > WALL_THRESHOLD:
        return -20
    
    # Reward moving forward in open space
    if action == "Forward" and front < OBSTACLE_THRESHOLD:
        return 3
    
    # Reward turning away from obstacles
    if (action in ["Sharp Left", "Slight Left"] and right > left):
        return 2
    if (action in ["Sharp Right", "Slight Right"] and left > right):
        return 2
    
    # Small penalty for unnecessary turns
    if action != "Forward" and front < OBSTACLE_THRESHOLD:
        return -1
    
    return 0  # Default

# Main loop
while robot.step(time_step) != -1:
    # Read sensors
    sensor_values = [sensor.getValue() for sensor in sensors]
    
    # Get current state
    state = get_state(sensor_values)
    
    # Check for collision (if any sensor detects a wall)
    collision_detected = any(v > WALL_THRESHOLD for v in sensor_values)
    
    # If collision detected, force a recovery action
    if collision_detected:
        # Stop, reverse briefly, then turn sharply
        left_motor.setVelocity(reverse_speed)
        right_motor.setVelocity(reverse_speed)
        robot.step(time_step * 5)  # Reverse for 5 time steps
        
        # Turn sharply in a random direction (left or right)
        if random.random() < 0.5:
            left_motor.setVelocity(-turn_speed)
            right_motor.setVelocity(turn_speed)
        else:
            left_motor.setVelocity(turn_speed)
            right_motor.setVelocity(-turn_speed)
        robot.step(time_step * 10)  # Turn for 10 time steps
        
        # Skip Q-learning update for this forced recovery
        continue
    
    # Choose action (only if no collision)
    action = choose_action(state)
    
    # Execute action
    if action == "Forward":
        left_motor.setVelocity(forward_speed)
        right_motor.setVelocity(forward_speed)
    elif action == "Sharp Left":
        left_motor.setVelocity(-turn_speed)
        right_motor.setVelocity(turn_speed)
    elif action == "Sharp Right":
        left_motor.setVelocity(turn_speed)
        right_motor.setVelocity(-turn_speed)
    elif action == "Slight Left":
        left_motor.setVelocity(forward_speed * 0.3)
        right_motor.setVelocity(forward_speed)
    elif action == "Slight Right":
        left_motor.setVelocity(forward_speed)
        right_motor.setVelocity(forward_speed * 0.3)
    
    # Get reward
    reward = get_reward(sensor_values, action)
    
    # Get next state
    next_state = get_state([sensor.getValue() for sensor in sensors])
    
    # Update Q-table
    update_Q(state, action, reward, next_state)
    
    # Optional: Print debug info
    # print(f"State: {state}, Action: {action}, Reward: {reward}, Epsilon: {epsilon:.3f}")