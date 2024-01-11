from controller import Robot, DistanceSensor, Motor, Camera
import numpy as np
import ctypes
import math

# Create the Robot instance.
robot = Robot()

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Global flag to indicate maze completion
maze_solved = False

Avoided = False

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
wall_sensor_names = ["w_front", "w_left", "w_right" , "sonarl" ,"sonarf"]
for name in wall_sensor_names:
    sensor = robot.getDevice(name)
    if sensor is None:
        print(f"Wall sensor '{name}' not found on robot 'youBot'. Please check the name.")
        continue
    sensor.enable(timestep)
    wall_sensors[name] = sensor

# Initialize box distance sensors.
box_sensors = {}
box_sensor_names = ["b_front", "b_front1", "b_front2"]
for name in box_sensor_names:
    sensor = robot.getDevice(name)
    if sensor is None:
        print(f"Box sensor '{name}' not found on robot 'youBot'. Please check the name.")
        continue
    sensor.enable(timestep)
    box_sensors[name] = sensor

#! Initialize arm motors.
armMotors = []
armMotors.append(robot.getDevice("arm1"))
armMotors.append(robot.getDevice("arm2"))
armMotors.append(robot.getDevice("arm3"))
armMotors.append(robot.getDevice("arm4"))
armMotors.append(robot.getDevice("arm5"))
# Set the maximum motor velocity.
armMotors[0].setVelocity(1.5) # maxVelocity = 1.5
armMotors[1].setVelocity(1.5)
armMotors[2].setVelocity(1.5)
armMotors[3].setVelocity(0.5)
armMotors[4].setVelocity(1.5)

#! Initialize arm position sensors.
# These sensors can be used to get the current 
# joint position and monitor the joint movements.
armPositionSensors = []
armPositionSensors.append(robot.getDevice("arm1sensor"))
armPositionSensors.append(robot.getDevice("arm2sensor"))
armPositionSensors.append(robot.getDevice("arm3sensor"))
armPositionSensors.append(robot.getDevice("arm4sensor"))
armPositionSensors.append(robot.getDevice("arm5sensor"))
for sensor in armPositionSensors:
    sensor.enable(timestep)

#! Initialize gripper motors.
finger1 = robot.getDevice("finger::left")
finger2 = robot.getDevice("finger::right")
# Set the maximum motor velocity.
finger1.setVelocity(1.5)
finger2.setVelocity(1.5) # 0.03
# Read the miminum and maximum position of the gripper motors.
fingerMinPosition = finger1.getMinPosition()
fingerMaxPosition = finger1.getMaxPosition()

####################################################################################
# Function to set wheel velocities

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
    
   # print(f"Setting velocities: Wheel1: {v1}, Wheel2: {v2}, Wheel3: {v3}, Wheel4: {v4}")
    
#####################################################################################################     

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
        print(position)
        orientation = obj.getOrientation()
        size = obj.getSize()
        
    ################# Print statements for debugging #################
    
        if color[0] == 1.0:
            print(f"Recognized object color: Red")
        elif color[1] == 1.0:
            print(f"Recognized object color: Green")
            PickupBox(obj)
        elif color[2] == 1.0:
            print(f"Recognized object color: Blue")
            AvoidBox(obj)
            
        # print(f"Recognized object color: {color[0]}, {color[1]}, {color[2]}")
  
  
        # print(f"Position: {position}")
        
        
        
#####################################################################################################     
#PID
    
def follow_line_pid():
    # PID coefficients
    Kp = 0.05  # Proportional gain
    Ki = 0.0   # Integral gain
    Kd = 0.02  # Derivative gain


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

    if left_value < line_threshold and middle_value < line_threshold and right_value < line_threshold:
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



# def maze_solver():
    # global maze_solved  
    # print("Starting maze solving.")

    # recognized_objects = camera.getRecognitionObjects()
    # while robot.step(timestep) != -1 and not maze_solved and not recognized_objects:  # Check the flag in the loop condition

        # front_dist = max(wall_sensors["w_front"].getValue(), 0)
        # left_dist = max(wall_sensors["w_left"].getValue(), 0)
        # middle_sensor_value = line_sensors[1].getValue() 
        
        # print("middle sensor value: ", middle_sensor_value)
     
        # if middle_sensor_value <= RED_AREA_THRESHOLD:
            # print("Red area detected. Maze solved!")
            # halt() 
            # maze_solved = True  
            # break  
            
        

        # if left_dist < 1000 and front_dist > SAFE_DISTANCE:
            # set_wheel_velocity(7.0, 7.0, 7.0, 7.0)


        # elif front_dist <= SAFE_DISTANCE:
            # set_wheel_velocity(-TURN_SPEED, TURN_SPEED, -TURN_SPEED, TURN_SPEED)
            # while robot.step(timestep) != -1 and max(wall_sensors["w_front"].getValue(), 0) <= SAFE_DISTANCE:

                # pass

            # set_wheel_velocity(7.0, 7.0, 7.0, 7.0)


        # elif left_dist == 1000:

            # set_wheel_velocity(0, 0, 0, 0)
            # robot.step(timestep) 
            

            # while robot.step(timestep) != -1 and max(wall_sensors["w_left"].getValue(), 0) >= 1000:
                # set_wheel_velocity(TURN_SPEED, -TURN_SPEED, TURN_SPEED, -TURN_SPEED)

            # set_wheel_velocity(7.0, 7.0, 7.0, 7.0)


        # robot.step(timestep)

######################################################

SAFE_DISTANCE = 600
f_speed = 10
RED_AREA_THRESHOLD = 650 

DEFAULT_DESIRED_DISTANCE = 400
DISTANCE_THRESHOLD = 50  # Adjust as needed
MAX_VELOCITY = 14  # Adjust as needed
TURNING_DESIRED_DISTANCE = 300  # Adjust as needed for turning

def adjust_speed(desired, actual):

    error = desired - actual
    k_p = 0.01  
    return max(-MAX_VELOCITY, min(MAX_VELOCITY, k_p * error))
    
def maze_solver():
    global maze_solved  # Declare the use of the global variable
    print("Starting maze solving.")
    
    while robot.step(timestep) != -1 and not maze_solved:  # Check the flag in the loop condition

        front_dist = max(wall_sensors["w_front"].getValue(), 0)
        left_dist = max(wall_sensors["w_left"].getValue(), 0)
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
            print("Sonar Front Distance: ", sonarf)
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
        print(f"Sensor Readings - Left: {left_dist}, Front: {front_dist} , Sonar Left: {sonarl} , Sonar Front: {sonarf}")
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
    
def turn_around(time):
#TODO: robot.step(70 * timestep) - produces a 90 degrees turn
    wheels[0].setVelocity(14)
    wheels[1].setVelocity(-14)
    wheels[2].setVelocity(14)
    wheels[3].setVelocity(-14)
    robot.step(70 * timestep)
    # forward()
    
def turn_around1():
    # Set wheels to turn around (180 degrees)
    set_wheel_velocity(TURN_SPEED, -TURN_SPEED, TURN_SPEED, -TURN_SPEED)
    robot.step(2 * TURN_DURATION * timestep)  # Double the duration for a full turn
    halt()  # Stop the robot after turning

# Function to turn the robot left for a specified duration
def turn_left_seconds(duration_seconds):
    set_wheel_velocity(10, -10, 10, -10)
    robot.step(int(duration_seconds * 1000 / timestep))  # Convert seconds to milliseconds
      # Stop the robot after turning

# Function to turn the robot right for a specified duration
def turn_right_seconds(duration_seconds):
    set_wheel_velocity(-10, 10, -10, 10)
    robot.step(int(duration_seconds * 1000 / timestep))  # Convert seconds to milliseconds
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
# Robotic Arm Movement Functions

def fold_arms():
    armMotors[0].setPosition(-2.9)
    armMotors[1].setPosition(1.5)
    armMotors[2].setPosition(-2.6)
    armMotors[3].setPosition(1.7)
    armMotors[4].setPosition(0)

def stretch_arms():
    armMotors[0].setPosition(2.9)
    armMotors[1].setPosition(-1.0)
    armMotors[2].setPosition(2.5)
    armMotors[3].setPosition(-1.7)
    armMotors[4].setPosition(0)



def pick_up():
    armMotors[1].setPosition(-1.1)
    armMotors[2].setPosition(-0.9)
    armMotors[3].setPosition(-1.1)
    finger1.setPosition(fingerMaxPosition)
    finger2.setPosition(fingerMaxPosition)

    # Monitor the arm joint position to 
    # detect when the motion is completed.
    while robot.step(timestep) != -1:
        if abs(armPositionSensors[3].getValue() - (-1.2)) < 0.1:
        # Motion completed.
            break
    print("CLOSING GRIPPER")
    finger1.setPosition(0.014)     # Close gripper.
    finger2.setPosition(0.014)
    print("GRIP CLOSED")
    robot.step(50 * timestep)    # Wait until the gripper is closed.
    armMotors[1].setPosition(0)    # Lift arm.
    # Wait until the arm is lifted.
    # robot.step(200 * timestep)
    
def open_grippers():
    finger1.setPosition(fingerMaxPosition)
    finger2.setPosition(fingerMaxPosition)
    robot.step(70 * timestep)

def drop():
    armMotors[1].setPosition(0.5)
    armMotors[3].setPosition(1.3)
    armMotors[2].setPosition(1)
    while robot.step(timestep) != -1:

        if abs(armPositionSensors[3].getValue() - (-1.2)) > 2.5:
        # Motion completed.
            print("READY TO OPEN GRIPPERS")
            break

def hand_up():
    armMotors[0].setPosition(0)
    armMotors[1].setPosition(0)
    armMotors[2].setPosition(0)
    armMotors[3].setPosition(0)
    armMotors[4].setPosition(0)
    finger1.setPosition(fingerMaxPosition)
    finger2.setPosition(fingerMaxPosition)  
    
######################################################################################    
# THE BOX CODE
######################################################################################    

def check_box_distance():

    box_detected = 0 

    while (box_detected == 0):

        b_dist_front = max(box_sensors["b_front"].getValue(), 0)
        b_dist_right = max(box_sensors["b_right"].getValue(), 0)
        print("FRONT BOX SENSOR: ", b_dist_front)
        print("RIGHT BOX SENSOR: ", b_dist_right)
        
        if b_dist_right < 1000:
            halt()
            print("STOPPED") 
            box_detected = 1
            turn_around(700)
            
        elif  b_dist_front < 120:
            halt()
            pick_up()
            drop()
            open_grippers()

######################################################################################    

def GoalFacing(): 
        
    recognized_object_array = camera.getRecognitionObjects()
    array_len = len(recognized_object_array)
    
    
    if array_len == 0:
        print('searching for goal...')
        
    else: 
        print('goal found!')
        recognized_object = camera.getRecognitionObjects()[0]
        image_position = recognized_object.getPositionOnImage()
        print("IMAGE POSITION" , image_position[0])
        

        if image_position[0] > 300:
            print("Turning Left")

            turn_right() #left spin
        if image_position[0] < 300:

            print("Turning Right")
            turn_left() #right spin
            
        if image_position[0] > 300 and image_position[0] < 350 :
            halt()
             
        elif image_position[0] == 300:
            
             forward(5)
         

        return [recognized_object.getPosition(),recognized_object.getOrientation(), recognized_object.getSize()]
  
            
       
######################################################################################    
# Pickup Box      
 
def PickupBox(box):
    box_detected = 0 
    going_back = False
    while (box_detected == 0):

        b_dist_front = max(box_sensors["b_front"].getValue(), 0)
                       
        print("FRONT BOX SENSOR: ", b_dist_front) 
             

        print('Box in sight! ')

        image_position = box.getPositionOnImage()           
        print("IMAGE POSITION" , image_position[0])

        if  b_dist_front < 80:
            going_back = True
            backward(50)
                     
        if  b_dist_front < 115 and going_back == False:
            halt()
            pick_up()
            drop()
            open_grippers()
            hand_up()
                        
        if image_position[0] > 300:
        
            print("Turning Right")
            turn_right() 
                      
        if image_position[0] < 300:

            print("Turning Left")
            turn_left() 
            
        if image_position[0] > 300 and image_position[0] < 350 :
            halt()
             
        elif image_position[0] == 300:
            
             forward(3)   
                      
        return [box.getPosition(),box.getOrientation(), box.getSize()]
       
######################################################################################    
# Avoiding the Box with the same stored color

def AvoidBox():
    global Avoided, robot, timestep # Declare the use of the global variable
    print("############\nAvoiding Loop.")
    while robot.step(timestep) != -1 and not Avoided:  # Check the flag in the loop condition
    
    
        b_front = max(box_sensors["b_front"].getValue(), 0)
        b_front1 = max(box_sensors["b_front1"].getValue(), 0)
        b_front2 = max(box_sensors["b_front2"].getValue(), 0)
        
        set_wheel_velocity(5.0, 5.0, 5.0, 5.0)
        
        # Parameters for obstacle avoidance
        safe_distance = 850  # Safe distance to maintain from the box
        turn_speed = 4.0     # Speed for turning
        turn_speed1 = 4.0     # Speed for turning
        forward_speed = 5.0  # Speed for moving forward
        turn_duration_right = 2.2  # Turning time to the right in seconds
        turn_duration_right1 = 2.5  # Turning time to the right in seconds
        turn_duration_forward = 1.5  # Turning time to more forward in seconds
        turn_duration_left = 4.0   # Turning time to the left in seconds
        turn_duration_left1 = 2.2   # Turning time to the left in seconds
    
    
        print("############\nAvoiding Box.")
        print(f"Avoided Value: ", str(Avoided))
        print(f"Sensors Values: \nLeft: {b_front1}\nRight: {b_front2}\nFront: {b_front}")
        
        # Calculate the number of simulation steps for the desired turn durations
        num_steps_to_turn_right = int((turn_duration_right * 1000) / timestep)
        num_steps_to_turn_right1 = int((turn_duration_right1 * 1000) / timestep)
        num_steps_to_turn_left = int((turn_duration_left * 1000) / timestep)
        num_steps_to_turn_left1 = int((turn_duration_left1 * 1000) / timestep)
        num_steps_to_turn_forward = int((turn_duration_forward * 1000) / timestep)
        
        if b_front < safe_distance or b_front1 < safe_distance:
            # Turn right for a set duration
            for _ in range(num_steps_to_turn_right):
                # Perform the turn
                set_wheel_velocity(forward_speed - turn_speed, forward_speed, forward_speed - turn_speed, forward_speed)
                # Step simulation to proceed to the next time step
                if robot.step(timestep) == -1:
                    break
            # After turning right, move forward for a set duration
            for _ in range(num_steps_to_turn_forward):
                set_wheel_velocity(forward_speed, forward_speed, forward_speed, forward_speed)
                # Step simulation to proceed to the next time step
                if robot.step(timestep) == -1:
                    break       
                    
             # After moving forward, turn left for a set duration
            for _ in range(num_steps_to_turn_left1):
                # Perform the turn
                # set_wheel_velocity(forward_speed - turn_speed, forward_speed, forward_speed - turn_speed, forward_speed)
                set_wheel_velocity(forward_speed, forward_speed - turn_speed, forward_speed, forward_speed - turn_speed)
                # Step simulation to proceed to the next time step
                if robot.step(timestep) == -1:
                    break
                    
            # move forward for a set duration
            for _ in range(num_steps_to_turn_forward):
                set_wheel_velocity(forward_speed, forward_speed, forward_speed, forward_speed)
                # Step simulation to proceed to the next time step
                if robot.step(timestep) == -1:
                    break     
                    
                    
            #################### Correct place ####################      
             # After turning left, turn left for a set duration
            for _ in range(num_steps_to_turn_left1):
                # Perform the turn
                # set_wheel_velocity(forward_speed - turn_speed, forward_speed, forward_speed - turn_speed, forward_speed)
                set_wheel_velocity(forward_speed, forward_speed - turn_speed, forward_speed, forward_speed - turn_speed)
                # Step simulation to proceed to the next time step
                if robot.step(timestep) == -1:
                    break
                    
             # Turn right for a set duration
            for _ in range(num_steps_to_turn_right):
                # Perform the turn
                set_wheel_velocity(forward_speed - turn_speed, forward_speed, forward_speed - turn_speed, forward_speed)
                print(f"AAAAAAAAAAAAAAAAAAAAAAAAA ")
                Avoided = True
                # Step simulation to proceed to the next time step
                if robot.step(timestep) == -1:
                    break
                
    #################### Extra time for the right turn because the box is on the right ####################         
    
        elif b_front2 < safe_distance:
            # Turn right for a set duration
            for _ in range(num_steps_to_turn_right1):
                # Perform the turn
                set_wheel_velocity(forward_speed - turn_speed, forward_speed, forward_speed - turn_speed, forward_speed)
                # Step simulation to proceed to the next time step
                if robot.step(timestep) == -1:
                    break
            # After turning right, move forward for a set duration
            for _ in range(num_steps_to_turn_forward):
                set_wheel_velocity(forward_speed, forward_speed, forward_speed, forward_speed)
                # Step simulation to proceed to the next time step
                if robot.step(timestep) == -1:
                    break       
                    
             # After moving forward, turn left for a set duration
            for _ in range(num_steps_to_turn_left1):
                # Perform the turn
                # set_wheel_velocity(forward_speed - turn_speed, forward_speed, forward_speed - turn_speed, forward_speed)
                set_wheel_velocity(forward_speed, forward_speed - turn_speed, forward_speed, forward_speed - turn_speed)
                # Step simulation to proceed to the next time step
                if robot.step(timestep) == -1:
                    break
                    
            # move forward for a set duration
            for _ in range(num_steps_to_turn_forward):
                set_wheel_velocity(forward_speed, forward_speed, forward_speed, forward_speed)
                # Step simulation to proceed to the next time step
                if robot.step(timestep) == -1:
                    break     
                    
                    
            #################### Correct place ####################      
             # After turning left, turn left for a set duration
            for _ in range(num_steps_to_turn_left1):
                # Perform the turn
                # set_wheel_velocity(forward_speed - turn_speed, forward_speed, forward_speed - turn_speed, forward_speed)
                set_wheel_velocity(forward_speed, forward_speed - turn_speed, forward_speed, forward_speed - turn_speed)
                # Step simulation to proceed to the next time step
                if robot.step(timestep) == -1:
                    break
                    
             # Turn right for a set duration
            for _ in range(num_steps_to_turn_right):
                # Perform the turn
                set_wheel_velocity(forward_speed - turn_speed, forward_speed, forward_speed - turn_speed, forward_speed)
                print(f"AAAAAAAAAAAAAAAAAAAAAAAAA ")
                Avoided = True
                
                # Step simulation to proceed to the next time step
                if robot.step(timestep) == -1:
                    break
                    
        # else:
            # If no box is detected within the safe distance, move forward
            # set_wheel_velocity(forward_speed, forward_speed, forward_speed, forward_speed)
            # Avoided = True

             
######################################################################################    

# first_object_color = None

# def process_camera_image_and_act():
    # global first_object_color
    # safe_distance = 850 
    # b_front = max(box_sensors["b_front"].getValue(), 0)
    # b_front1 = max(box_sensors["b_front1"].getValue(), 0)
    # b_front2 = max(box_sensors["b_front2"].getValue(), 0)

    # recognized_objects = camera.getRecognitionObjects()
    

    # if not recognized_objects:
        # if not line_follower:
               # follow_line_pid()
        # elif not maze_solved:
                # maze_solver()
        # return


    # for obj in recognized_objects:
       
        # color_ptr = obj.getColors()
        

        # color_array = ctypes.cast(color_ptr, ctypes.POINTER(ctypes.c_double * 3))
        

        # color = color_array.contents


        # if first_object_color is None:
            # if b_front < safe_distance or b_front1 < safe_distance or b_front2 < safe_distance:
                # first_object_color = (color[0], color[1], color[2])
                # print(f"Detected first object, Color is:", str(first_object_color))
              
                # AvoidBox()
                # return  
                
            # else:
                # if not line_follower:
                    # print("line follower L" , line_follower ) 
                    # follow_line_pid()
                # elif not maze_solved:

                    # maze_solver()
            
        # else:
            
            # if b_front < safe_distance or b_front1 < safe_distance or b_front2 < safe_distance:
                # if (color[0], color[1], color[2]) == first_object_color:
               

                    # print(f"Same Color")
                    # AvoidBox()
                    # return
                
            
            # elif (color[0], color[1], color[2]) != first_object_color:

                # AnotherFunction(obj)
                
            # else:
                    # if not line_follower:
                        # follow_line_pid()
                    # elif not maze_solved:
                        # maze_solver()   

        
# def AnotherFunction(obj):

    # print(f"I SHOULD GRAB THAT")
    # pass

######################################################################################
while robot.step(timestep) != -1:
    if maze_solved:
        print("Exiting the program.")
        break  

    #process_camera_image_and_act()

    # AvoidBox()
    follow_line_pid()  
    #process_camera_image()


   
