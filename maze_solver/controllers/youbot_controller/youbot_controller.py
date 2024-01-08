from controller import Robot, DistanceSensor, Motor, Camera
import numpy as np
import ctypes

# Create the Robot instance.
robot = Robot()

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Global flag to indicate maze completion
maze_solved = False

######################################################################################    
#INITIALIZATIONS

# Initialize the camera.
camera = robot.getDevice("cam")
camera.enable(timestep)
camera.recognitionEnable(timestep)

# Initialize base motors.
wheels = []
wheel_names = ["wheel1", "wheel2", "wheel3", "wheel4"]
for name in wheel_names:
    wheel = robot.getDevice(name)
    if wheel is None:
        print(f"Device '{name}' not found on robot 'youBot'. Please check the name.")
        continue
    wheel.setPosition(float('+inf'))  # Infinite position for velocity control
    wheels.append(wheel)

# Initialize line following sensors.
line_sensors = []
line_sensor_names = ["ls_left", "ls_middle", "ls_right"]
for name in line_sensor_names:
    sensor = robot.getDevice(name)
    if sensor is None:
        print(f"Line sensor '{name}' not found on robot 'youBot'. Please check the name.")
        continue
    sensor.enable(timestep)
    line_sensors.append(sensor)

# Initialize wall distance sensors.
wall_sensors = {}
wall_sensor_names = ["w_front", "w_left", "w_right"]
for name in wall_sensor_names:
    sensor = robot.getDevice(name)
    if sensor is None:
        print(f"Wall sensor '{name}' not found on robot 'youBot'. Please check the name.")
        continue
    sensor.enable(timestep)
    wall_sensors[name] = sensor

   
####################################################################################
# Function to set wheel velocities

def set_wheel_velocity(v1, v2, v3, v4):
    wheels[0].setVelocity(v1)
    wheels[1].setVelocity(v2)
    wheels[2].setVelocity(v3)
    wheels[3].setVelocity(v4)
    
    ################# Print statements for debugging #################
    print(f"Setting velocities: Wheel1: {v1}, Wheel2: {v2}, Wheel3: {v3}, Wheel4: {v4}")

#####################################################################################################     
#PID

def follow_line_pid():
    # PID coefficients
    Kp = 0.05  # Proportional gain
    Ki = 0.0   # Integral gain
    Kd = 0.02  # Derivative gain

    # PID variables
    integral = 0
    last_error = 0
    base_velocity = 8.0  # Base speed of the robot

    # Retrieve sensor values
    left_value = line_sensors[0].getValue()
    middle_value = line_sensors[1].getValue()
    right_value = line_sensors[2].getValue()

    # print(f"Sensor Readings - Left: {left_value}, Middle: {middle_value}, Right: {right_value}")

    # Define threshold values
    line_threshold = 1000
    off_line_threshold = 968

    # Check if robot needs to move straight forward
    if middle_value == line_threshold:
        set_wheel_velocity(base_velocity, base_velocity, base_velocity, base_velocity)
        return
    elif left_value < line_threshold and middle_value < line_threshold and right_value < line_threshold:
        # Robot entered the maze, exit follow_line_pid and start solving
        maze_solver()
        return
    elif left_value < off_line_threshold:
        if right_value == line_threshold:
            # Sharp turn to the right
            set_wheel_velocity(-7.0, 7.0, 0.0, 0.0)
            # print("sharp turn to the right")
            return
        else:
            error = off_line_threshold - left_value
    elif right_value < off_line_threshold:
        # Sharp turn to the left
        set_wheel_velocity(7.0, -7.0, 0.0, 0.0)
        return
    else:
        # Calculate error based on the deviation from the line
        error = right_value - left_value

    # PID calculations
    P = error
    integral += error
    I = integral
    D = error - last_error
    pid_output = Kp * P + Ki * I + Kd * D
    last_error = error

    # Set wheel velocities based on PID output
    left_wheel_velocity = base_velocity + pid_output
    right_wheel_velocity = base_velocity - pid_output

    # Apply new velocities to the wheels
    set_wheel_velocity(left_wheel_velocity, left_wheel_velocity, right_wheel_velocity, right_wheel_velocity)

#######################################################
#MAZE SOLVING

SAFE_DISTANCE = 500

RED_AREA_THRESHOLD = 1000 # Define the threshold value for the red area

def maze_solver():
    global maze_solved  # Declare the use of the global variable
    
    print("Starting maze solving.")
    while robot.step(timestep) != -1 and not maze_solved:  # Check the flag in the loop condition

        front_dist = max(wall_sensors["w_front"].getValue(), 0)
        left_dist = max(wall_sensors["w_left"].getValue(), 0)
        middle_sensor_value = line_sensors[1].getValue() # This is the ls_middle sensor value
        
        print("middle sensor value: ", middle_sensor_value)
        # Check if the middle sensor detects the red area
        if middle_sensor_value == RED_AREA_THRESHOLD:
            print("Red area detected. Maze solved!")
            halt()  # Stop the robot
            maze_solved = True  # Set the flag to indicate the maze is solved
            break  # Break out of the while loop
            
        
        # If there's a wall to the left and space in front, move forward
        if left_dist < 1000 and front_dist > SAFE_DISTANCE:
            set_wheel_velocity(7.0, 7.0, 7.0, 7.0)

        # If there's a wall in front, turn right
        elif front_dist <= SAFE_DISTANCE:
            set_wheel_velocity(-TURN_SPEED, TURN_SPEED, -TURN_SPEED, TURN_SPEED)
            while robot.step(timestep) != -1 and max(wall_sensors["w_front"].getValue(), 0) <= SAFE_DISTANCE:
                # Turning in place until there's space in front
                pass
            # Correct orientation to face down the new corridor
            set_wheel_velocity(7.0, 7.0, 7.0, 7.0)

        # If the left sensor value equals 1000, stop and turn left in place
        elif left_dist == 1000:
            # Stop the robot first
            set_wheel_velocity(0, 0, 0, 0)
            robot.step(timestep) # Update sensors before starting to turn
            
            # Turn left in place until the left sensor detects the wall again
            while robot.step(timestep) != -1 and max(wall_sensors["w_left"].getValue(), 0) >= 1000:
                set_wheel_velocity(TURN_SPEED, -TURN_SPEED, TURN_SPEED, -TURN_SPEED)
            # Once the wall is detected, the robot can continue moving forward
            set_wheel_velocity(7.0, 7.0, 7.0, 7.0)

        # Perform a step to update sensor readings after each action
        robot.step(timestep)






######################################################

def move_forward():
    print(f"forward")
    set_wheel_velocity(10.0, 10.0, 10.0, 10.0)

# Constants
TURN_DURATION = 5 
TURN_SPEED = 7.0    
###########################
SLIGHT_TURN_DURATION = 3  
SLIGHT_TURN_SPEED = 3.0    

def slight_left_turn():
    # Set wheels for a slight left turn while moving forward
    set_wheel_velocity(10.0 + SLIGHT_TURN_SPEED, 10.0 - SLIGHT_TURN_SPEED, 10.0 + SLIGHT_TURN_SPEED, 10.0 - SLIGHT_TURN_SPEED)
    # set_wheel_velocity(10.0 - SLIGHT_TURN_SPEED, 10.0 + SLIGHT_TURN_SPEED, 10.0 - SLIGHT_TURN_SPEED, 10.0 + SLIGHT_TURN_SPEED)


def slight_right_turn():
    # Set wheels for a slight right turn while moving forward
    set_wheel_velocity(10.0 + SLIGHT_TURN_SPEED, 10.0 - SLIGHT_TURN_SPEED, 10.0 + SLIGHT_TURN_SPEED, 10.0 - SLIGHT_TURN_SPEED)
    
def turn_left():
    # Set wheels to turn left
    set_wheel_velocity(TURN_SPEED, -TURN_SPEED, TURN_SPEED, -TURN_SPEED)
    robot.step(TURN_DURATION * timestep)
    halt()  # Stop the robot after turning

def turn_right():
    # Set wheels to turn right
    set_wheel_velocity(-TURN_SPEED, TURN_SPEED, -TURN_SPEED, TURN_SPEED)
    robot.step(TURN_DURATION * timestep)
    halt()  # Stop the robot after turning

def turn_around():
    # Set wheels to turn around (180 degrees)
    set_wheel_velocity(TURN_SPEED, -TURN_SPEED, TURN_SPEED, -TURN_SPEED)
    robot.step(2 * TURN_DURATION * timestep)  # Double the duration for a full turn
    halt()  # Stop the robot after turning

def halt():
    set_wheel_velocity(0, 0, 0, 0)  # Stop all wheels

######################################################################################    
    
# Movement functions
def forward(time):
    for wheel in wheels:
        wheel.setVelocity(7.0)  # Adjust max velocity as needed
    robot.step(time * timestep)

def backward(time):
    for wheel in wheels:
        wheel.setVelocity(-7.0)  # Adjust max velocity as needed
    robot.step(time * timestep)

    
######################################################################################    

# Main robot control loop
while robot.step(timestep) != -1:
    if maze_solved:
        print("Exiting the program.")
        break  # Exit the loop if the maze is solved
    
    follow_line_pid()  # Follow the line
   
