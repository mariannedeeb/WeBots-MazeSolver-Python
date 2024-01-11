from controller import Robot, DistanceSensor, Motor, Camera
import numpy as np
import ctypes
import time 
import math

# Create the Robot instance.
robot = Robot()

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Global flag to indicate maze completion
maze_solved = False

integral = 0
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
wall_sensor_names = ["w_front", "w_left", "w_right", "front_left", "rear_left","sonarl","sonarf"]
for name in wall_sensor_names:
    sensor = robot.getDevice(name)
    if sensor is None:
        print(f"Wall sensor '{name}' not found on robot 'youBot'. Please check the name.")
        continue
    sensor.enable(timestep)
    wall_sensors[name] = sensor

   
####################################################################################
# # Function to set wheel velocities

MAX_VELOCITY = 14.81  # Maximum allowed wheel velocity

def set_wheel_velocity(v1, v2, v3, v4):
    # Clamp each wheel's velocity to be within the maximum limit
    v1 = max(min(v1, MAX_VELOCITY), -MAX_VELOCITY)
    v2 = max(min(v2, MAX_VELOCITY), -MAX_VELOCITY)
    v3 = max(min(v3, MAX_VELOCITY), -MAX_VELOCITY)
    v4 = max(min(v4, MAX_VELOCITY), -MAX_VELOCITY)

    wheels[0].setVelocity(v1)
    wheels[1].setVelocity(v2)
    wheels[2].setVelocity(v3)
    wheels[3].setVelocity(v4)
    
    #print(f"Setting velocities: Wheel1: {v1}, Wheel2: {v2}, Wheel3: {v3}, Wheel4: {v4}")
#####################################################################################################     
#PID
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

    print(f"Sensor Readings - Left: {left_value}, Middle: {middle_value}, Right: {right_value}")

    # Define threshold values
    line_threshold = 1000
    off_line_threshold = 968

    # Check if robot needs to move straight forward
    # if middle_value == line_threshold:
        # set_wheel_velocity(base_velocity, base_velocity, base_velocity, base_velocity)
        # return
    if left_value < line_threshold and middle_value < line_threshold and right_value < line_threshold:
        # Robot entered the maze, exit follow_line_pid and start solving
        maze_solverrr()
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

SAFE_DISTANCE = 600
f_speed = 10
RED_AREA_THRESHOLD = 650 # Define the threshold value for the red area

def maze_solver():
    global maze_solved  # Declare the use of the global variable
    print("Starting maze solving.")
    while robot.step(timestep) != -1 and not maze_solved:  # Check the flag in the loop condition

        front_dist = max(wall_sensors["w_front"].getValue(), 0)
        left_dist = max(wall_sensors["w_left"].getValue(), 0)
        rear_left = max(wall_sensors["rear_left"].getValue(), 0)
        front_left = max(wall_sensors["front_left"].getValue(), 0)
        middle_sensor_value = line_sensors[1].getValue() # This is the ls_middle sensor value
        
        # print("middle sensor value: ", middle_sensor_value)
        # Check if the middle sensor detects the red area
        if middle_sensor_value <= RED_AREA_THRESHOLD:
            print("Red area detected. Maze solved!")
            halt()  # Stop the robot
            maze_solved = True  # Set the flag to indicate the maze is solved
            break  # Break out of the while loop
            
############################ MOVING CASES ############################ 
        # If there's a wall to the left and space in front, move forward
        if left_dist < 1000 and front_dist > SAFE_DISTANCE:
            print("FIRST IF - FORWARD")
            print(f"Sensor Readings - Left: {left_dist}, Front: {front_dist}")
            set_wheel_velocity(f_speed, f_speed, f_speed, f_speed)

        # If there's a wall in front, turn right
        elif front_dist <= SAFE_DISTANCE and left_dist < 1000:
            print(f"Sensor Readings - Left: {left_dist}, Front: {front_dist}")
            print("2ND IF - TURN RIGHT")
            print("STOP ROBOT")
            # Stop the robot first
            set_wheel_velocity(0, 0, 0, 0)
            robot.step(timestep) # Update sensors before starting to turn
            
            while robot.step(timestep) != -1 and max(wall_sensors["w_front"].getValue(), 0) < 1000 and abs(front_left - rear_left) > 5:
                # Turning in place until there's space in front
                print("2ND IF - TURN RIGHT")
                print(f"Sensor Readings - Left: {left_dist}, Front: {front_dist}")
                set_wheel_velocity(-TURN_SPEED, 0, -TURN_SPEED, 0)
                # pass
            # Correct orientation to face down the new corridor
            set_wheel_velocity(f_speed, f_speed, f_speed, f_speed)

        # If the left sensor value equals 1000, stop and turn left in place
        elif left_dist == 1000:
            print(f"Sensor Readings - Left: {left_dist}, Front: {front_dist}")
            print("3RD IF - TURN LEFT")
            print("STOP ROBOT")
            # Stop the robot first
            set_wheel_velocity(0, 0, 0, 0)
            robot.step(timestep) # Update sensors before starting to turn
            
            # Turn left in place until the left sensor detects the wall again
            while robot.step(timestep) != -1 and max(wall_sensors["w_left"].getValue(), 0) >= 1000 :
                print(f"Sensor Readings - Left: {left_dist}, Front: {front_dist}")
                print("TURN LEFT")
                set_wheel_velocity(0, -TURN_SPEED, 0, -TURN_SPEED)
            # Once the wall is detected, the robot can continue moving forward
            set_wheel_velocity(f_speed, f_speed, f_speed, f_speed)

        # Perform a step to update sensor readings after each action
        print(f"Sensor Readings - Left: {left_dist}, Front: {front_dist}")
        robot.step(timestep)






########



# Constants for the wall following behavior
# SAFE_DISTANCE_FRONT = 500  # Safe distance from the front wall (in mm)
# SAFE_DISTANCE_LEFT = 500   # Safe distance from the left wall (in mm)
# TURN_SPEED = 3.0           # Speed of the robot when turning
# FORWARD_SPEED = 5.0        # Speed of the robot when moving forward

# def maze_solver():
    # global maze_solved
    # Initialize PID variables
    # last_error = 0
    # integral = 0
    # Kp = 0.05  # Proportional gain
    # Ki = 0.0   # Integral gain
    # Kd = 0.02  # Derivative gain

    # print("Starting maze solving.")
    # while robot.step(timestep) != -1 and not maze_solved:
        # front_dist = max(wall_sensors["w_front"].getValue(), 0)
        # left_dist = max(wall_sensors["w_left"].getValue(), 0)
        # middle_sensor_value = line_sensors[1].getValue()

        # print("Sensor Readings - Left: {}, Front: {}".format(left_dist, front_dist))

        # Check if the middle sensor detects the red area
        # if middle_sensor_value <= RED_AREA_THRESHOLD:
            # print("Red area detected. Maze solved!")
            # halt()
            # maze_solved = True
            # break

        # Wall following logic
        # if front_dist < SAFE_DISTANCE_FRONT:
            # Turn right if an obstacle is detected in front
            # set_wheel_velocity(-TURN_SPEED, TURN_SPEED, -TURN_SPEED, TURN_SPEED)
        # else:
            # Adjust the robot's course based on the left distance sensor
            # error = left_dist - SAFE_DISTANCE_LEFT
            # P = error
            # integral += error
            # pid_output = Kp * P + Ki * integral + Kd * (error - last_error)
            # last_error = error

            # Apply new velocities to the wheels
            # left_wheel_velocity = FORWARD_SPEED + pid_output
            # right_wheel_velocity = FORWARD_SPEED - pid_output
            # set_wheel_velocity(left_wheel_velocity, left_wheel_velocity, right_wheel_velocity, right_wheel_velocity)

######################################################
#Hrayr's Edition

DEFAULT_DESIRED_DISTANCE = 400
DISTANCE_THRESHOLD = 50  # Adjust as needed
MAX_VELOCITY = 14  # Adjust as needed
TURNING_DESIRED_DISTANCE = 300  # Adjust as needed for turning

def adjust_speed(desired, actual):
    # Proportional control to adjust the speed based on the distance difference
    error = desired - actual
    k_p = 0.01  # Proportional gain, adjust as needed
    return max(-MAX_VELOCITY, min(MAX_VELOCITY, k_p * error))
    
def maze_solverrr():
    global maze_solved  # Declare the use of the global variable
    print("Starting maze solving.")
    
    while robot.step(timestep) != -1 and not maze_solved:  # Check the flag in the loop condition

        front_dist = max(wall_sensors["w_front"].getValue(), 0)
        left_dist = max(wall_sensors["w_left"].getValue(), 0)
        rear_left = max(wall_sensors["rear_left"].getValue(), 0)
        front_left = max(wall_sensors["front_left"].getValue(), 0)
        sonarl = max(wall_sensors["sonarl"].getValue(), 0)
        sonarf = max(wall_sensors["sonarf"].getValue(), 0)

        middle_sensor_value = line_sensors[1].getValue()  # This is the ls_middle sensor value

        # print("middle sensor value: ", middle_sensor_value)
        # Check if the middle sensor detects the red area
        if middle_sensor_value <= RED_AREA_THRESHOLD:
            print("Red area detected. Maze solved!")
            halt()  # Stop the robot
            maze_solved = True  # Set the flag to indicate the maze is solved
            break  # Break out of the while loop

        ############################ MOVING CASES ############################

        # Calculate the adjustment to the left wheel speed based on the distance error
        left_speed_adjustment = adjust_speed(400, left_dist)
  
        # If there's a wall to the left and space in front, move forward with left speed adjustment
        
        
        if left_dist < 1000 and sonarl < 1000 and front_dist > SAFE_DISTANCE:
            print("Sonar Front Distance: ",sonarf)
            print("FORWARD - Adjusted Left Speed:", left_speed_adjustment)
            set_wheel_velocity(f_speed , f_speed + left_speed_adjustment, f_speed , f_speed + left_speed_adjustment)
                
        # If there's a wall in front, turn right
        elif front_dist <= SAFE_DISTANCE and left_dist < 1000 and sonarf < 1000:
            print("TURN RIGHT")
            set_wheel_velocity(0, 0, 0, 0)
            robot.step(timestep)  # Update sensors before starting to turn

            while robot.step(timestep) != -1 and max(wall_sensors["w_front"].getValue(), 0) < 1000:
                print("TURN RIGHT - Adjusted Left Speed:", left_speed_adjustment)
                set_wheel_velocity(-14, 14, -14, 14)
                
        # If the left sensor value equals 1000, stop and turn left in place
        elif left_dist == 1000 and sonarl == 1000:
            print("Sonar Distance: ",sonarl)
            set_wheel_velocity(0, 0, 0, 0)
            robot.step(timestep)  # Update sensors before starting to turn

            # Adjust desired distance for turning
            DEFAULT_DESIRED_DISTANCE = min(TURNING_DESIRED_DISTANCE, left_dist - DISTANCE_THRESHOLD)

            # Turn left in place until the left sensor detects the wall again
            while robot.step(timestep) != -1 and max(wall_sensors["w_left"].getValue(), 0) >= 900:
                print("TURN LEFT - Adjusted Left Speed:", left_speed_adjustment)
                print(f"Sensor Readings - Left: {left_dist}, Front: {front_dist} , Sonar Left: {sonarl}")

                set_wheel_velocity(14, -14, 14, -14)

            # Once the wall is detected, the robot can continue moving forward
            set_wheel_velocity(f_speed + left_speed_adjustment, f_speed, f_speed + left_speed_adjustment, f_speed)
            DEFAULT_DESIRED_DISTANCE = 400  # Reset the desired distance after turning

        # Perform a step to update sensor readings after each action
        print(f"Sensor Readings - Left: {left_dist}, Front: {front_dist} , Sonar Left: {sonarl}")
        robot.step(timestep)
        
        
        
        
########################################################
def move_forward():
    print(f"forward")
    set_wheel_velocity(10.0, 10.0, 10.0, 10.0)

# Constants
TURN_DURATION = 5 
TURN_SPEED = 5.0    
# ###########################
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
    # halt()  # Stop the robot after turning

def turn_right():
    # Set wheels to turn right
    set_wheel_velocity(-TURN_SPEED, TURN_SPEED, -TURN_SPEED, TURN_SPEED)
    robot.step(TURN_DURATION * timestep)
    # halt()  # Stop the robot after turning

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

def slight_left():
    set_wheel_velocity(2, -2, -2, 2)

def slight_right():
    set_wheel_velocity(-2, 2, 2, -2)
    
 
######################################################################################    

# Main robot control loop
while robot.step(timestep) != -1:
    if maze_solved:
        print("Exiting the program.")
        break  # Exit the loop if the maze is solved
    
    follow_line_pid()  # Follow the line

