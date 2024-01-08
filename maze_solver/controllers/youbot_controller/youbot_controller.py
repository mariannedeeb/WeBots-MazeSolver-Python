from controller import Robot, DistanceSensor, Motor, Camera
import numpy as np
# import cv2
import ctypes

# Create the Robot instance.
robot = Robot()

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

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
wall_sensor_names = ["w_front", "w_back", "w_left", "w_right"]
for name in wall_sensor_names:
    sensor = robot.getDevice(name)
    if sensor is None:
        print(f"Wall sensor '{name}' not found on robot 'youBot'. Please check the name.")
        continue
    sensor.enable(timestep)
    wall_sensors[name] = sensor

# Initialize arm motors.
armMotors = []
armMotorNames = ["arm1", "arm2", "arm3", "arm4", "arm5"]
for name in armMotorNames:
    motor = robot.getDevice(name)
    motor.setVelocity(1.5)  # Set a suitable velocity
    armMotors.append(motor)

# Initialize arm position sensors.
armPositionSensors = []
armPositionSensorNames = ["arm1sensor", "arm2sensor", "arm3sensor", "arm4sensor", "arm5sensor"]
for name in armPositionSensorNames:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    armPositionSensors.append(sensor)
   
####################################################################################


# Function to set wheel velocities
def set_wheel_velocity(v1, v2, v3, v4):
    wheels[0].setVelocity(v1)
    wheels[1].setVelocity(v2)
    wheels[2].setVelocity(v3)
    wheels[3].setVelocity(v4)
    
    ################# Print statements for debugging #################
    # print(f"Setting velocities: Wheel1: {v1}, Wheel2: {v2}, Wheel3: {v3}, Wheel4: {v4}")

# Process and analyze camera image and recognized objects
def process_camera_image():
    recognized_objects = camera.getRecognitionObjects()
    for obj in recognized_objects:
        # Get the pointer to the color array
        color_ptr = obj.getColors()

        # Convert the pointer to an array of three doubles
        color_array = ctypes.cast(color_ptr, ctypes.POINTER(ctypes.c_double * 3))

        # Access the color values
        color = color_array.contents
        
        # Get object's position, orientation, and size
        position = obj.getPosition()
        orientation = obj.getOrientation()
        size = obj.getSize()
        
    ################# Print statements for debugging #################
    
        if color[0] == 1.0:
            print(f"Recognized object color: Red")
        elif color[1] == 1.0:
            print(f"Recognized object color: Green")
        elif color[2] == 1.0:
            print(f"Recognized object color: Blue")
            
        # print(f"Recognized object color: {color[0]}, {color[1]}, {color[2]}")
        # print(f"Position: {position}")
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

######################################################################################
# Constants
# SAFE_DISTANCE = 300 

# def maze_solver():
    # print("Starting maze solving.")
    # while robot.step(timestep) != -1:

        # front_dist = max(wall_sensors["w_front"].getValue(), 0)
        # left_dist = max(wall_sensors["w_left"].getValue(), 0)
        # right_dist = max(wall_sensors["w_right"].getValue(), 0)


        # print(f"Front distance: {front_dist}, Left distance: {left_dist}, Right distance: {right_dist}")


        # if front_dist > SAFE_DISTANCE:
            # print("Moving forward")
            # move_forward()
        # elif right_dist > left_dist:
            # print("Turning right")
            # turn_right()
        # elif left_dist > 0:
            # print("Turning left")
            # turn_left()
        # else:
            # print("Turning around")
            # turn_around()


        # robot.step(10 * timestep)
#######################################################
#HRAYR'S FUNCTION

SAFE_DISTANCE = 500

# Constants for maintaining straight path (you can adjust these values)
BALANCE_THRESHOLD = 70  # Adjust this threshold as needed
BALANCE_SPEED = 1.0  # Adjust the balancing speed as needed

def maze_solver():
    print("Starting maze solving.")
    while robot.step(timestep) != -1:

        front_dist = max(wall_sensors["w_front"].getValue(), 0)
        left_dist = max(wall_sensors["w_left"].getValue(), 0)
        right_dist = max(wall_sensors["w_right"].getValue(), 0)

        print(f"Left distance: {left_dist},Front distance: {front_dist}, Right distance: {right_dist}")

        if front_dist > SAFE_DISTANCE:
            # Calculate the difference between left and right distances
            diff = left_dist - right_dist

            # Adjust wheel velocities to balance the distances
            left_wheel_speed = 7.0
            right_wheel_speed = 7.0
            print(f"forward")

            if diff > BALANCE_THRESHOLD:
                left_wheel_speed -= BALANCE_SPEED
                right_wheel_speed += BALANCE_SPEED
                print(f"turn left")
            elif diff < -BALANCE_THRESHOLD:
                left_wheel_speed += BALANCE_SPEED
                right_wheel_speed -= BALANCE_SPEED
                print(f"turn right")
            # Set balanced wheel velocities
            set_wheel_velocity(left_wheel_speed, left_wheel_speed, right_wheel_speed, right_wheel_speed)
            
        elif right_dist > SAFE_DISTANCE:
            print("Turning right11")
            turn_right()    
            
        elif left_dist > SAFE_DISTANCE:
            print("Turning left11")
            turn_left() 
            
        elif right_dist > left_dist:
            print("Turning right")
            turn_right()
            
        elif left_dist > right_dist:
            print("Turning left")
            turn_left()
            
        else:
            print("Turning around")
            turn_around()
        

        robot.step(15 * timestep)


######################################################
#HRAYR 2


# def adjust_to_wall():

    # right_dist = wall_sensors["w_right"].getValue()
    # left_dist = wall_sensors["w_left"].getValue()

    # Target distance from the right wall
    # target_distance = 200  # Adjust this as needed

    # Proportional gain for adjustment
    # Kp = 0.05

    # Calculate the difference from the target distance
    # difference = right_dist - target_distance

    # Calculate the adjustment value
    # adjustment = Kp * difference

    # Adjust wheel speeds to maintain distance from the right wall
    # left_wheel_speed = 7.0 - adjustment
    # right_wheel_speed = 7.0 + adjustment

    # set_wheel_velocity(left_wheel_speed, right_wheel_speed, left_wheel_speed, right_wheel_speed)

    # Debug print
    # print(f"Adjusting to wall. Wheel speeds: Left: {left_wheel_speed}, Right: {right_wheel_speed}")




#######################################################
def move_forward():
    print(f"forward")
    set_wheel_velocity(10.0, 10.0, 10.0, 10.0)

# Constants
TURN_DURATION = 5 # Adjust this value based on trial and error
TURN_SPEED = 7.0    # Adjust the turning speed as needed

# Constants for slight turns
SLIGHT_TURN_DURATION = 3  # Adjust this value based on trial and error
SLIGHT_TURN_SPEED = 3.0    # Adjust the turning speed as needed

def slight_left_turn():
    # Set wheels for a slight left turn while moving forward
    set_wheel_velocity(10.0 - SLIGHT_TURN_SPEED, 10.0 + SLIGHT_TURN_SPEED, 10.0 - SLIGHT_TURN_SPEED, 10.0 + SLIGHT_TURN_SPEED)


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

def halt():
    for wheel in wheels:
        wheel.setVelocity(0.0)

# Arm control functions (if needed)
def pick_up():
    pass  # Implement the pick-up functionality here

def drop():
    pass  # Implement the drop functionality here

# Main robot control loop
while robot.step(timestep) != -1:
    follow_line_pid()  # Follow the line
    # Insert conditions or triggers to call other functions like pick_up, drop, etc.
   
    # Process and analyze the camera image
    process_camera_image()
