#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import math


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()

# --- Robot Hardware Configuration ---
Back_ARM = Motor(Port.B,positive_direction=Direction.CLOCKWISE)
left_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE) 
right_motor = Motor(Port.D, Direction.CLOCKWISE)
Front_motor = Motor(Port.A,Direction.CLOCKWISE)     

# Initialize TWO color sensors for line following
line_left = ColorSensor(Port.S2)  # Left sensor
line_right = ColorSensor(Port.S1) # Right sensor

color_sensor = ColorSensor(Port.S3)
gyro_sensor = GyroSensor(Port.S4)


detected_colors = [] #For four color payload color store

Axle_track=200
Wheel_Diameter = 55
robot = DriveBase(left_motor, right_motor, wheel_diameter=Wheel_Diameter, axle_track=Axle_track)
BackArm_UP_POSITION_MOTOR_ANGLE = Back_ARM.angle()    # BackGrabber Angle
#ev3.screen.print(BackArm_UP_POSITION_MOTOR_ANGLE)
BackArm_Down_POSITION_MOTOR_ANGLE = 260
Front_wingsUp_POSITION_MOTOR_ANGLE = Front_motor.angle() 
print(Front_wingsUp_POSITION_MOTOR_ANGLE)
Front_wingsDown_POSITION_MOTOR_ANGLE  = -400
# Define turn parameters
#TURN_ANGLE = 90 # The desired angle for the turn
TURN_SPEED = 400  # The speed of the motors in degrees/second

# --- State Machine Configuration ---
# Define the possible states for the robot
STATE_LINE_FOLLOWING_TO_DISTANCE = 1  # Main state: line follow, speed ramp, check distance
STATE_MISSION_BARRIER_COLOR = 2       # State for the final mission
STATE_MISSION_SLIDE = 3  
STATE_MISSION_COLORSENSE = 4
STATE_STOP = 5      # Final stop state  
STATE_RETURN_LINE_FOLLOWING = 6  
STATE_SQUARE_PLACING = 7
STATE_FLAG_MISSION = 8  
STATE_SQUARE_GRABBING = 9 
STATE_PAYLOAD_PICK_UP =10
STATE_PAYLOAD_SORT =11
STATE_PAYLOAD_PLACING_I = 12
STATE_PAYLOAD_PLACING_II = 14
STATE_PUSHING_BARRIER =15
STATE_RETURN_LINE_FOLLOWING_II = 13         
#STATE_INTERSECTION_DETECTED = 4 


#For speed up
INITIAL_LINE_FOLLOW_SPEED = 200 # Starting speed for line following (mm/s)
MAX_LINE_FOLLOW_SPEED = 500    # Maximum desired speed for line following (mm/s) - this was your old DRIVE_SPEED
SPEED_INCREMENT_LF = 10         # How much to increase speed each step (mm/s)
RAMP_UP_DELAY_LF_MS = 50       # Delay between each speed increment (milliseconds)

#For slowdown
SLOW_DOWN_START_DISTANCE = 300 #(30cm before the target)
MIN_DRIVE_SPEED = 50 # Minimum speed to slow down to before stopping
TARGET_DISTANCE_MM = 1820 # 156 cm


# Set the initial state of the robot
current_state = STATE_LINE_FOLLOWING_TO_DISTANCE

# --- State Variables ---
intersection_count = 0 # This one didn't use in this program yet
is_first_line_follow_segment = True

ev3.speaker.beep()
ev3.screen.clear()
#Gyro initial Read
# Gyro initial Read
gyro_sensor.reset_angle(0)
initial_heading = gyro_sensor.angle()
print(initial_heading)
ev3.speaker.beep()


# --- PID Line Follower Parameters ---
BLACK_CALIBRATED_VALUE = 12
WHITE_CALIBRATED_VALUE = 55

KP = 0.1 # Proportional Gain: Tune first.
KI = 0.001 # Integral Gain: Tune second (start very small).
KD = 0.005 # Derivative Gain: Tune third (start relatively small).

# Integral Windup Prevention Limits
INTEGRAL_MAX = 500
INTEGRAL_MIN = -500

DRIVE_SPEED = INITIAL_LINE_FOLLOW_SPEED # Robot's constant forward speed (mm/s)
MAX_STEERING_RATE = 250 # Max allowed turn rate (degrees/second)
MIN_STEERING_RATE = -250

# PID calculation variables
error = 0         # Current error (difference between sensor readings)
last_error = 0    # Error from the previous loop iteration
integral = 0      # Accumulated sum of errors over time
derivative = 0    # Rate of change of error

state_initialization_done = False #

# -------------------------------------------------------------
## Gyro Turning Precision
# -------------------------------------------------------------
KP_turn = 20 # Proportional gain (Adjust this first)
KI_turn = 0.0001# Integral gain (Adjust this last, keep small)
KD_turn = 5# Derivative gain (Adjust this second)

# Integral Windup Prevention Limits for Turning
INTEGRAL_MAX_TURN = 100
INTEGRAL_MIN_TURN = -100
TURN_SPEED_DPS = 500

# -------------------------------------------------------------
## Pivot Turning GRYO PID
# -------------------------------------------------------------
def precise_gyro_pivot_turn(initial_heading, turn_angle):
    # Set the absolute target angle based on the initial heading and the turn angle.
    absolute_target_angle = initial_heading + turn_angle

    integral_turn = 0
    last_error_turn = 0
    TURN_SPEED_DPS = 500  # You can adjust this speed for the pivot turn

    #ev3.screen.print("Pivoting to:", absolute_target_angle)

    while True:
        current_angle = gyro_sensor.angle()
        error_turn = absolute_target_angle - current_angle

        # Proportional term (P)
        P_turn = KP_turn * error_turn

        # Integral term (I) with windup prevention
        integral_turn += error_turn
        integral_turn = max(min(integral_turn, INTEGRAL_MAX_TURN), INTEGRAL_MIN_TURN)
        if (error_turn > 0 and last_error_turn < 0) or \
           (error_turn < 0 and last_error_turn > 0):
            integral_turn = 0
        I_turn = KI_turn * integral_turn

        # Derivative term (D)
        derivative_turn = error_turn - last_error_turn
        D_turn = KD_turn * derivative_turn

        # Total PID steering value
        steering = P_turn + I_turn + D_turn
        steering = max(min(steering, TURN_SPEED_DPS), -TURN_SPEED_DPS)

        # Apply power to motors for a pivot turn
        # For a clockwise pivot turn (e.g., turning right)
        if turn_angle > 0:
            left_motor.run(steering)
            right_motor.run(0)  # Right motor stays still
        # For a counter-clockwise pivot turn (e.g., turning left)
        else:
            left_motor.run(0)   # Left motor stays still
            right_motor.run(-steering)
        
        # Break condition
        if abs(error_turn) < 1 and abs(steering) < 5:
            break
            
        last_error_turn = error_turn
        wait(10)

    left_motor.brake()
    right_motor.brake()
    wait(50)
    ev3.speaker.beep()

# -------------------------------------------------------------
## Pivot Turning GRYO PID Backward
# -------------------------------------------------------------
def precise_gyro_backward_pivot_turn(initial_heading, turn_angle):
    absolute_target_angle = initial_heading + turn_angle
    print(absolute_target_angle)
    integral_turn = 0
    last_error_turn = 0
    TURN_SPEED_DPS = 500

    #ev3.screen.print("Backward pivoting to:", absolute_target_angle)

    while True:
        current_angle = gyro_sensor.angle()
        error_turn = absolute_target_angle - current_angle
        print("Error Turn",error_turn)
        P_turn = KP_turn * error_turn
        
        integral_turn += error_turn
        integral_turn = max(min(integral_turn, INTEGRAL_MAX_TURN), INTEGRAL_MIN_TURN)
        if (error_turn > 0 and last_error_turn < 0) or \
           (error_turn < 0 and last_error_turn > 0):
            integral_turn = 0
        I_turn = KI_turn * integral_turn
        
        derivative_turn = error_turn - last_error_turn
        D_turn = KD_turn * derivative_turn
        
        steering = P_turn + I_turn + D_turn
        steering = max(min(steering, TURN_SPEED_DPS), -TURN_SPEED_DPS)

        # Corrected Motor Control Logic
        # If the error is positive, the robot needs to turn clockwise. 
        # The left motor must move backward.
        if error_turn > 0:
            left_motor.run(0)
            right_motor.run(-abs(steering)) # Corrected
        # If the error is negative, the robot needs to turn counter-clockwise.
        # The left motor must move backward.
        else:
            left_motor.run(-abs(steering)) # Corrected
            right_motor.run(0)
        
        if abs(error_turn) < 1 and abs(steering) < 5:
            break
            
        last_error_turn = error_turn
        wait(5)

    left_motor.brake()
    right_motor.brake()
    wait(50)
    ev3.speaker.beep()

# -------------------------------------------------------------
## Spin Turning GRYO PID
# ------------------------------------------------------------- 

def precise_gyro_turn_from_start(initial_heading, turn_angle):

    absolute_target_angle = initial_heading + turn_angle
    
    integral_turn = 0
    last_error_turn = 0
    
    #ev3.screen.print("Turning to:", absolute_target_angle)

    while True:
        current_angle = gyro_sensor.angle()
        error_turn = absolute_target_angle - current_angle
        
        # --- Proportional Term (P) ---
        P_turn = KP_turn * error_turn
        
        # --- Integral Term (I) with Windup Prevention ---
        integral_turn += error_turn
        # Clamp the integral to prevent it from growing too large
        integral_turn = max(min(integral_turn, INTEGRAL_MAX_TURN), INTEGRAL_MIN_TURN)
        
        # **NEW**: Reset integral on overshoot to correct faster
        # This checks if the error has crossed zero (changed sign)
        if (error_turn > 0 and last_error_turn < 0) or \
           (error_turn < 0 and last_error_turn > 0):
            integral_turn = 0
            
        I_turn = KI_turn * integral_turn
        
        # --- Derivative Term (D) ---
        derivative_turn = error_turn - last_error_turn
        D_turn = KD_turn * derivative_turn
        
        # --- Total PID Steering Value ---
        steering = P_turn + I_turn + D_turn
        
        # Limit the steering value to the maximum motor speed
        steering = max(min(steering, TURN_SPEED_DPS), -TURN_SPEED_DPS)

        # Apply power to motors for a spin turn
        left_motor.run(steering)
        right_motor.run(-steering)
        
        # --- Break Condition: Check if we are stable at the target ---
        # Stop when the error is tiny and the robot has stopped trying to correct
        if abs(error_turn) < 1 and abs(steering) < 5:
            break
            
        last_error_turn = error_turn
        wait(10) # Small delay for stability
        
    left_motor.brake()
    right_motor.brake()
    wait(50) # Pause briefly to ensure the stop is complete
    ev3.speaker.beep()

# -------------------------------------------------------------
## Speed Management Functions
# -------------------------------------------------------------
#Gradual Speed-UP logic
def manage_speed_up():
    #Gradually increases DRIVE_SPEED up to MAX_LINE_FOLLOW_SPEED.
    global DRIVE_SPEED
    if DRIVE_SPEED < MAX_LINE_FOLLOW_SPEED: # This condition is essential!
        DRIVE_SPEED += SPEED_INCREMENT_LF
        # Cap the speed at the maximum in case the increment overshoots slightly
        if DRIVE_SPEED > MAX_LINE_FOLLOW_SPEED:
            DRIVE_SPEED = MAX_LINE_FOLLOW_SPEED
        # Add a small delay specific to the acceleration, if desired.
        wait(RAMP_UP_DELAY_LF_MS) # This delay controls the acceleration rate


#Speed Ramp Down Logic
def manage_speed_down(current_distance_traveled):
    #Gradually decreases DRIVE_SPEED as the robot approaches Target.
    global DRIVE_SPEED, TARGET_DISTANCE_MM, SLOW_DOWN_START_DISTANCE, MAX_LINE_FOLLOW_SPEED, MIN_DRIVE_SPEED
    distance_remaining = TARGET_DISTANCE_MM - current_distance_traveled
    if distance_remaining <= SLOW_DOWN_START_DISTANCE and distance_remaining > 0:
    # Calculate a new speed based on remaining distance
    # 'progress' from 0 (at SLOW_DOWN_START_DISTANCE) to 1 (at 0 distance remaining)
        progress = (SLOW_DOWN_START_DISTANCE - distance_remaining) / SLOW_DOWN_START_DISTANCE
    # Calculate new speed
        new_speed = MAX_LINE_FOLLOW_SPEED - (progress * (MAX_LINE_FOLLOW_SPEED - MIN_DRIVE_SPEED))
        DRIVE_SPEED = max(new_speed, MIN_DRIVE_SPEED) # Ensure speed doesn't go below MIN_DRIVE_SPEED
    elif distance_remaining <= 0: # If we passed the target, ensure speed is minimum
        DRIVE_SPEED = MIN_DRIVE_SPEED
    else: # If far from target, maintain max speed
        DRIVE_SPEED = MAX_LINE_FOLLOW_SPEED # Ensure we are at max speed initially


# -------------------------------------------------------------
## Back Arm Motor Functions
# -------------------------------------------------------------
def go_grab_down(speed):
    global BackArm_Down_POSITION_MOTOR_ANGLE
    #Moves the grab system to the calibrated DOWN (ground) position.
    Back_ARM.run_target(speed,BackArm_Down_POSITION_MOTOR_ANGLE, then=Stop.HOLD, wait=True)

def go_grab_up(speed):
    global BackArm_UP_POSITION_MOTOR_ANGLE # Access the global variable
    #if BackArm_UP_POSITION_MOTOR_ANGLE == 0:
       # ev3.screen.print("Arm not calibrated (UP)!")
       # return
    #ev3.screen.print("Arm Going Up...")
    #Moves the grab system to the calibrated UP position.
    Back_ARM.run_target(speed,BackArm_UP_POSITION_MOTOR_ANGLE, then=Stop.HOLD, wait=True)
    #ev3.screen.print("Arm Up!")

# -------------------------------------------------------------
## Wings Up and Down (THE FRONT MECHENISM)
# ------------------------------------------------------------- 

def wings_down(speed):
    global Front_wingsDown_POSITION_MOTOR_ANGLE 
    Front_motor.run_target(speed, Front_wingsDown_POSITION_MOTOR_ANGLE, then=Stop.HOLD, wait=True)

def wings_up(speed):
    global Front_wingsUp_POSITION_MOTOR_ANGLE
    #ev3.screen.print("Arm Going Up...")
    Front_motor.run_target(speed, Front_wingsUp_POSITION_MOTOR_ANGLE, then=Stop.HOLD, wait=True)
    #ev3.screen.print("Arm Up!")
# -------------------------------------------------------------
## #######   State Machine Functions ####################
# -------------------------------------------------------------
# ------------------------------------------------------------
## State: Line Following
# -------------------------------------------------------------
def line_follow_to_target_distance():
    global current_state, integral, last_error, reflection_left, reflection_right, DRIVE_SPEED,DISTANCE_TO_BARRIER ,state_initialization_done, error, derivative, is_first_line_follow_segment# Added current_drive_speed
    
    if not state_initialization_done: # Check the global flag
        robot.reset() # Reset distance at the beginning of this drive segment
        integral = 0  # Reset PID integral
        last_error = 0 # Reset PID last error
        DRIVE_SPEED = INITIAL_LINE_FOLLOW_SPEED # Start speed from initial for ramp-up
        #ev3.screen.print("Line follow to:", TARGET_DISTANCE_MM, "mm")
        state_initialization_done = True # Set the global flag to True
    
    # Read Sensor Inputs
    # Get current reflected light intensity from both sensors.
    reflection_left = line_left.reflection()
    reflection_right = line_right.reflection()

     # Get current distance traveled
    current_distance = robot.distance() # Get the current distance from the robot

    #---------PID Calculation ----------
    # Calculate the Error Term (Crucial for two sensors)
    # The error is the difference between the two sensor readings.
    error = reflection_left - reflection_right

    # Calculate Proportional (P) Term
    P_term = KP * error

    # Calculate Integral (I) Term
    integral = integral + error
    integral = max(min(integral, INTEGRAL_MAX), INTEGRAL_MIN) # Integral Windup Prevention
    I_term = KI * integral

    # Calculate Derivative (D) Term
    derivative = error - last_error
    D_term = KD * derivative

    # Calculate Total Turn Rate
    turn_rate = P_term + I_term + D_term

    # Limit the Turn Rate
    turn_rate = max(min(turn_rate, MAX_STEERING_RATE), MIN_STEERING_RATE)

    # --- Call Speed-Up Logic ---
    if current_distance < (TARGET_DISTANCE_MM - SLOW_DOWN_START_DISTANCE):
        manage_speed_up()
    else:
        manage_speed_down(current_distance) # Start slowing down as target is approached

    # Command Robot Movement
    robot.drive(DRIVE_SPEED, turn_rate)

    # Update last_error for the next iteration
    last_error = error

    # Check for Target Distance Reached to transition state
    if current_distance >= TARGET_DISTANCE_MM:
        robot.stop()
        ev3.speaker.beep()
        ev3.screen.print("\nTarget Distance Reached!")
        state_initialization_done = False 
        if is_first_line_follow_segment:
            is_first_line_follow_segment = False
            current_state = STATE_MISSION_BARRIER_COLOR
        elif current_state == STATE_RETURN_LINE_FOLLOWING:
            #current_state = STATE_STOP
            current_state = STATE_SQUARE_GRABBING # Stop after the return trip is done
        elif current_state == STATE_RETURN_LINE_FOLLOWING_II:
            current_state = STATE_PAYLOAD_PLACING_II
        else:
            current_state = STATE_STOP # Transition to mission
    wait(5)
# -------------------------------------------------------------

# -------------------------------------------------------------
## Placing Two Bars 
# ------------------------------------------------------------- 
def mission_barrier_color():
    global current_state, TARGET_DISTANCE_MM, initial_heading,Front_wingsUp_POSITION_MOTOR_ANGLE# Allow modificaton of TARGET_DISTANCE_MM
    wings_down(800)
    robot.stop()
    wait(200)
    precise_gyro_turn_from_start(initial_heading, 90)
    initial_heading = gyro_sensor.angle()

    ######## Placing the Yellow Bar
    robot.stop() 
    robot.settings(straight_speed=600)
    robot.straight(-420)
    robot.stop()
    precise_gyro_turn_from_start(initial_heading, -45)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(-120)
    robot.stop()
    ev3.speaker.beep()
    wings_up(800)
    robot.straight(-40)
    wings_down(800)

    #Placing The Red Bar
    robot.straight(-100)
    robot.stop()
    precise_gyro_turn_from_start(initial_heading, -90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(110)
    robot.stop()
    wings_up(800)

    # Go to Red Slider
    robot.straight(-120)
    robot.stop()
    precise_gyro_turn_from_start(initial_heading, 45)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(-150) 
    robot.stop()
    precise_gyro_turn_from_start(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    ev3.speaker.beep()
    
    # Sliding Barrier Back Mechanism
    robot.straight(450)
    robot.stop()
    precise_gyro_backward_pivot_turn(initial_heading, -60)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    precise_gyro_backward_pivot_turn(initial_heading, 60)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.settings(straight_speed=400)

    robot.straight(-150)# Backward to slide Red Slider
    robot.stop()
    go_grab_down(500)
    robot.straight(150)
    robot.stop()
    robot.straight(-20)
    robot.stop()
    go_grab_up(500)
    precise_gyro_pivot_turn(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    
    # Find the line back before changing to next mission
    FORWARD_SPEED_TO_LINE = 200
    robot.drive(FORWARD_SPEED_TO_LINE, 0) # Drive straight forward
    # Loop until both sensors detect black
    while not (line_left.reflection() < BLACK_CALIBRATED_VALUE and line_right.reflection() < BLACK_CALIBRATED_VALUE):
        wait(10)
    robot.stop()
    left_motor.brake()
    right_motor.brake()
    wait(100)
    robot.stop()

    robot.straight(70)
    robot.stop()
    precise_gyro_pivot_turn(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    
    state_initialization_done = False
    current_state = STATE_MISSION_COLORSENSE
    #current_state = STATE_STOP

# ------------------------------------------------------------
## Function: Function for color sensing
# -------------------------------------------------------------
REF_RED = (4, 0, 0)
REF_BLUE = (0, 0, 6)
REF_GREEN = (0, 1, 0)
REF_YELLOW = (6, 3, 0)

def get_color(sensor):
    """
    Measures the current RGB value and compares it to reference values
    to determine the color.
    """
    # Get the current RGB reading
    current_rgb = sensor.rgb()
    r, g, b = current_rgb

    # Calculate the "distance" from each reference color
    # The smaller the distance, the closer the match
    distance_to_red = math.sqrt((r - REF_RED[0])**2 + (g - REF_RED[1])**2 + (b - REF_RED[2])**2)
    distance_to_blue = math.sqrt((r - REF_BLUE[0])**2 + (g - REF_BLUE[1])**2 + (b - REF_BLUE[2])**2)
    distance_to_green = math.sqrt((r - REF_GREEN[0])**2 + (g - REF_GREEN[1])**2 + (b - REF_GREEN[2])**2)
    distance_to_yellow = math.sqrt((r - REF_YELLOW[0])**2 + (g - REF_YELLOW[1])**2 + (b - REF_YELLOW[2])**2)

    # Find the smallest distance
    min_distance = min(distance_to_red, distance_to_blue, distance_to_green,distance_to_yellow)

    # Return the corresponding color
    if min_distance == distance_to_red:
        return "RED"
    elif min_distance == distance_to_blue:
        return "BLUE"
    elif min_distance == distance_to_green:
        return "GREEN"
    else: 
        return "YELLOW"
    

# ------------------------------------------------------------
## State: Color Sensing Version 2
# -------------------------------------------------------------

def mission_colorsense():
    global current_state, state_initialization_done, initial_heading, detected_colors,detected_color_Square,TARGET_DISTANCE_MM

    if not state_initialization_done:
        #move Forward abit to reach the color
        robot.settings(straight_speed=100)
        robot.straight(50)
        robot.stop()

        #color reading 
        detected_color_Square  = get_color(color_sensor)
        ev3.speaker.beep()
        #ev3.speaker.say ("Detected: {}".format(detected_color_Square)) #speak out the color
        wait(200)

        #will go to pushing berrier
        #Bumping to reset
        robot.settings(straight_speed=400)
        robot.straight(-570)
        robot.stop()
        ev3.speaker.beep()
        # Reset motor encoders after a maneuver
        left_motor.reset_angle(0)
        right_motor.reset_angle(0)
        initial_heading = gyro_sensor.angle()
        print(initial_heading)
        robot.stop()

        #Forward Until Black
        FORWARD_SPEED_TO_LINE = 300
        robot.drive(FORWARD_SPEED_TO_LINE, 0) # Drive straight forward
        # Loop until both sensors detect black
        while not (line_left.reflection() < BLACK_CALIBRATED_VALUE and line_right.reflection() < BLACK_CALIBRATED_VALUE):
            wait(10)
        robot.stop()
        
        ###################
        state_initialization_done = False
        #TARGET_DISTANCE_MM = 850
        #current_state = STATE_STOP
        #current_state = STATE_RETURN_LINE_FOLLOWING
        current_state = STATE_PUSHING_BARRIER

# -------------------------------------------------------------
## Pusing Barrier
# -------------------------------------------------------------

def pushing_barrier():
    global current_state, state_initialization_done, initial_heading,TARGET_DISTANCE_MM
    #new one
    precise_gyro_turn_from_start(initial_heading, -90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.settings(straight_speed=400)
    robot.straight(400) 
    robot.stop()
    precise_gyro_turn_from_start(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    precise_gyro_pivot_turn(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.settings(straight_speed=300)
    robot.straight(100) 
    robot.stop()
    precise_gyro_pivot_turn(initial_heading, -30)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    precise_gyro_backward_pivot_turn(initial_heading, 30)
    initial_heading = gyro_sensor.angle()
    robot.stop()

    #state_initialization_done = False
    #TARGET_DISTANCE_MM = 800
    #current_state = STATE_STOP
    current_state = STATE_FLAG_MISSION
    #current_state = STATE_RETURN_LINE_FOLLOWING_II

# -------------------------------------------------------------
## Flag Mission
# ------------------------------------------------------------- 
def mission_flag():
    global current_state, state_initialization_done, initial_heading,BackArm_Down_POSITION_MOTOR_ANGLE
    #new maneuver
    robot.straight(-450) 
    robot.stop()
    precise_gyro_turn_from_start(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    #Bumping Forward and setting
    robot.settings(straight_speed=350)
    robot.straight(280) 
    robot.stop()
    ev3.speaker.beep()
    # Reset motor encoders after a maneuver
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    #Go backward toward Flag
    robot.straight(-650) 
    robot.stop()
    go_grab_down(500)
    wait(100)
    go_grab_up(500)
    
    state_initialization_done = False
    #current_state = STATE_SQUARE_GRABBING
    current_state = STATE_STOP
   

# ------------------------------------------------------------
## Function: Function for different square color
# -------------------------------------------------------------

def red():
    global initial_heading
    robot.straight(50)
    robot.stop()
    precise_gyro_turn_from_start(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    #Bumping and setting
    robot.settings(straight_speed=400)
    robot.straight(-200) 
    robot.stop()
    ev3.speaker.beep()
    # Reset motor encoders after a maneuver
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(330) 
    robot.stop()
    precise_gyro_pivot_turn(initial_heading, -90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    wings_down(300)
    #Aim to square
    precise_gyro_pivot_turn(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(50) 
    robot.stop()
    precise_gyro_turn_from_start(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    #Bumping and setting
    robot.straight(-300) 
    robot.stop()
    ev3.speaker.beep()
    # Reset motor encoders after a maneuver
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    initial_heading = gyro_sensor.angle()
    robot.stop()

def green():
    global initial_heading
    robot.settings(straight_speed=400)
    robot.straight(230) 
    robot.stop()
    ev3.speaker.beep()
    precise_gyro_turn_from_start(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    #Bumping and setting
    robot.settings(straight_speed=400)
    robot.straight(-200) 
    robot.stop()
    ev3.speaker.beep()
    # Reset motor encoders after a maneuver
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(260) 
    robot.stop()
    wings_down(300)
    robot.straight(100) 
    robot.stop()
    precise_gyro_turn_from_start(initial_heading, -90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(-150) 
    wings_up(300)
    robot.stop()
    precise_gyro_backward_pivot_turn(initial_heading, 270)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(80)
    robot.stop()
    precise_gyro_turn_from_start(initial_heading, 180)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(20)
    robot.stop()
    wings_down(300)
    robot.straight(370) 
    robot.stop()
    precise_gyro_turn_from_start(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    #Bumping and setting
    robot.settings(straight_speed=400)
    robot.straight(-300) 
    robot.stop()
    ev3.speaker.beep()
    # Reset motor encoders after a maneuver
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    initial_heading = gyro_sensor.angle()
    robot.stop()

def blue():
    global initial_heading
    robot.settings(straight_speed=400)
    robot.straight(230) 
    robot.stop()
    precise_gyro_turn_from_start(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    #Bumping and setting
    robot.settings(straight_speed=400)
    robot.straight(-250) 
    robot.stop()
    ev3.speaker.beep()
    # Reset motor encoders after a maneuver
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(320) 
    robot.stop()
    wings_down(300)
    #Aim to square
    robot.straight(260) 
    robot.stop()
    precise_gyro_turn_from_start(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    #Bumping and setting
    robot.settings(straight_speed=400)
    robot.straight(-250) 
    robot.stop()
    ev3.speaker.beep()
    # Reset motor encoders after a maneuver
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    initial_heading = gyro_sensor.angle()
    robot.stop()

def yellow():
    global initial_heading
    robot.straight(50) 
    robot.stop()
    precise_gyro_turn_from_start(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    #Bumping and setting
    robot.settings(straight_speed=400)
    robot.straight(-180) 
    robot.stop()
    ev3.speaker.beep()
    # Reset motor encoders after a maneuver
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(550) 
    robot.stop()
    precise_gyro_pivot_turn(initial_heading, -180)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    wings_down(300)
    robot.straight(-20) 
    robot.stop()
    #Aim to square
    precise_gyro_turn_from_start(initial_heading, -90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    #precise_gyro_backward_pivot_turn(initial_heading, -90)
    #initial_heading = gyro_sensor.angle()
    #robot.stop()
    #Bumping and setting
    robot.straight(-250) 
    robot.stop()
    ev3.speaker.beep()
    # Reset motor encoders after a maneuver
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    initial_heading = gyro_sensor.angle()
    robot.stop()

# -------------------------------------------------------------
## Grabbing the Square Block due to Color
# -------------------------------------------------------------
def Grabbing_Square_Block():
    global current_state, state_initialization_done, initial_heading, detected_colors,detected_color_Square,TARGET_DISTANCE_MM
    #detected_color_Square ='RED'
    print(detected_color_Square)
    if detected_color_Square == 'RED':
        red()
    elif detected_color_Square == 'GREEN':
        green()
    elif detected_color_Square == 'BLUE':
        blue()
    else:
        yellow()

    state_initialization_done = False
    #current_state = STATE_STOP
    current_state = STATE_SQUARE_PLACING
# -------------------------------------------------------------
## Mission Square Placing
# ------------------------------------------------------------- 
def mission_square_placing():
    global current_state, state_initialization_done, initial_heading
    FORWARD_SPEED_TO_LINE = 300
    robot.drive(FORWARD_SPEED_TO_LINE, 0) # Drive straight forward
    # Loop until both sensors detect black
    while not (line_left.reflection() < BLACK_CALIBRATED_VALUE and line_right.reflection() < BLACK_CALIBRATED_VALUE):
        wait(10)
    robot.stop()
    left_motor.brake()
    right_motor.brake()
    wait(100)
    print("Line Found!")
    robot.stop()
    robot.straight(10)
    robot.stop()
    wings_up(300)
    state_initialization_done = False
    current_state = STATE_STOP
    #current_state = STATE_PAYLOAD_PICK_UP


# -------------------------------------------------------------
## Picking Up the PayLoads
# -------------------------------------------------------------
def pay_load_pick_up():
    global current_state, state_initialization_done, initial_heading
   
    precise_gyro_pivot_turn(initial_heading, -90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.settings(straight_speed=200)
    robot.straight(100)
    robot.stop()
    wings_down(300)
    robot.straight(-200)
    robot.stop()
   
    state_initialization_done = False
    current_state = STATE_STOP
    #current_state = STATE_PAYLOAD_PLACING_I
# -------------------------------------------------------------
## Pay Load Placing I
# -------------------------------------------------------------
def pay_load_placing_I():
    global current_state, state_initialization_done, initial_heading
    precise_gyro_turn_from_start(initial_heading, -90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    #Bumping and setting
    robot.straight(300) 
    robot.stop()
    ev3.speaker.beep()
    # Reset motor encoders after a maneuver
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(-50) 
    robot.stop()
    precise_gyro_backward_pivot_turn(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    FORWARD_SPEED_TO_LINE = 200
    robot.drive(FORWARD_SPEED_TO_LINE, 0) # Drive straight forward
    # Loop until both sensors detect black
    while not (line_left.reflection() < BLACK_CALIBRATED_VALUE and line_right.reflection() < BLACK_CALIBRATED_VALUE):
        wait(10)
    robot.stop()
    left_motor.brake()
    right_motor.brake()
    wait(100)
    robot.stop()
    pushing_barrier()
    wings_up(300)

    state_initialization_done = False
    current_state = STATE_PUSHING_BARRIER



# -------------------------------------------------------------
## Pay load placing II
# -------------------------------------------------------------

def pay_load_placing_II():
    global current_state, state_initialization_done, initial_heading
    precise_gyro_turn_from_start(initial_heading, -90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    #Bumping and setting
    robot.straight(800) 
    robot.stop()
    ev3.speaker.beep()
    # Reset motor encoders after a maneuver
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(-50) 
    robot.stop()
    precise_gyro_backward_pivot_turn(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(-100) 
    robot.stop()

    state_initialization_done = False
    current_state = STATE_STOP



# -------------------------------------------------------------
## Main Loop
# ------------------------------------------------------------- 
while True:
    ev3.screen.print(current_state)
    if current_state == STATE_LINE_FOLLOWING_TO_DISTANCE:
        line_follow_to_target_distance()
    elif current_state == STATE_MISSION_BARRIER_COLOR:
        mission_barrier_color()
    elif current_state == STATE_MISSION_SLIDE:
        mission_slide()
    elif current_state == STATE_MISSION_COLORSENSE:
        mission_colorsense()
    elif current_state == STATE_RETURN_LINE_FOLLOWING:
        line_follow_to_target_distance()
    elif current_state == STATE_SQUARE_GRABBING:
        Grabbing_Square_Block()
    elif current_state == STATE_SQUARE_PLACING:
        mission_square_placing()
    elif current_state == STATE_FLAG_MISSION:
        mission_flag()
    elif current_state == STATE_PAYLOAD_PICK_UP:
        pay_load_pick_up()
    elif current_state == STATE_PAYLOAD_PLACING_I:
        pay_load_placing_I()
    elif current_state ==STATE_PUSHING_BARRIER:
        pushing_barrier()
    elif current_state == STATE_RETURN_LINE_FOLLOWING_II:
        line_follow_to_target_distance()
    elif current_state == STATE_PAYLOAD_PLACING_II:
        pay_load_placing_II()
    elif current_state == STATE_STOP:
        robot.stop()
        ev3.screen.clear()
        ev3.screen.print("Mission Complete!")
        break
    wait(10)
