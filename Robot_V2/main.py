#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import math

# --- Setup ---
ev3 = EV3Brick()
#----------------------------------------------------------------------#
# --- Motors Declaration ---#
#----------------------------------------------------------------------#
Back_ARM = Motor(Port.B, positive_direction=Direction.CLOCKWISE)
left_motor = Motor(Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D, positive_direction=Direction.CLOCKWISE)
Front_ARM = Motor(Port.A, positive_direction=Direction.CLOCKWISE)

#----------------------------------------------------------------------#
# --- Sensors Declaration ---#
#----------------------------------------------------------------------#
line_left = ColorSensor(Port.S2)
line_right = ColorSensor(Port.S1)
# Optional room sensor (uncomment if connected)
color_sensor = ColorSensor(Port.S3)
gyro_sensor = GyroSensor(Port.S4)
gyro_sensor.reset_angle(0)

#----------------------------------------------------------------------#
# --- Robot Parameters ---#
#----------------------------------------------------------------------#
wheel_diameter = 56
axle_track = 223
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)

#----------------------------------------------------------------------#
# --- Arm parameters ---#
#----------------------------------------------------------------------#
BackArm_UP_POSITION_MOTOR_ANGLE = Back_ARM.angle()
BackArm_DOWN_POSITION_MOTOR_ANGLE = 125

FrontArm_UP_POSITION = 170
FrontArm_DOWN_POSITION = -350# adjust experimentally

#----------------------------------------------------------------------#
# --- State Declaration ---#
#----------------------------------------------------------------------#
STATE_LINE_FOLLOWING_TO_DISTANCE = 1 #LINE FOLLOWING
STATE_MISSION_BARRIER_COLOR = 2 # Two RED and YELLOW BAR TAKING AND PLACING
STATE_BARRIER_RED_SLIDING =3
STATE_TAKING_BLACK_BLOCK_MISSION = 4
STATE_PUSHING_FLAG = 5
STATE_PAYLOAD_TAKING = 6 # For Taking Payloads 
STATE_PAYLOAD_PLACING = 7 # Placing the payloads
STATE_BARRIER_SLIDING = 8
STATE_PAYLOAD_FINAL = 9
STATE_PAYLOAD_COLOR_SWITCHING = 10 # This state is for further use to switch the color of payloads before placing
STATE_STOP = 11
STATE_COLOR_SENSING =12
STATE_LINE_FOLLOW_AFTER_BLACK_BLOCK = 13
STATE_FIRST_PAYLOAD_PLACING =14
STATE_SECOND_PICKING = 15
STATE_LINE_FOLLOW_TO_BARRIER_PUSH =16
STATE_BARRIER_PUSH = 17
STATE_SECOND_PLACING =18
 
#----------------------------------------------------------------------#
# --- Line Following Parameters ---#
#----------------------------------------------------------------------#
INITIAL_LINE_FOLLOW_SPEED = 400
MAX_LINE_FOLLOW_SPEED = 800
SPEED_INCREMENT_LF = 10
RAMP_UP_DELAY_LF_MS = 50

SLOW_DOWN_START_DISTANCE = 400
MIN_DRIVE_SPEED = 50
TARGET_DISTANCE_MM = 1780 # start box to yellow barrier pusher
TARGET_DISTANCE_MM_2 = 900 # barrier to two bars

BLACK_CALIBRATED_VALUE = 12
WHITE_CALIBRATED_VALUE = 55

KP = 0.4
KI = 0.001
KD = 0.3

INTEGRAL_MAX = 500
INTEGRAL_MIN = -500

MAX_STEERING_RATE = 250
MIN_STEERING_RATE = -250

#----------------------------------------------------------------------#
# --- Initial parameters ---#
#----------------------------------------------------------------------#
current_state = STATE_LINE_FOLLOWING_TO_DISTANCE
DRIVE_SPEED = INITIAL_LINE_FOLLOW_SPEED
integral = 0
last_error = 0
state_initialization_done = False
last_scanned_color = None
detected_colors = []
gyro_sensor.reset_angle(0)
initial_heading = gyro_sensor.angle()
print(initial_heading)
ev3.speaker.beep()

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
    elif min_distance == distance_to_yellow:
        return "YELLOW"
    else:
        return "UNKNOWN"

#----------------------------------------------------------------------#
# --- Line follower Speed Management ( Not use yet) ---#
#----------------------------------------------------------------------#
def manage_speed_up():
    global DRIVE_SPEED
    if DRIVE_SPEED < MAX_LINE_FOLLOW_SPEED:
        DRIVE_SPEED += SPEED_INCREMENT_LF
        if DRIVE_SPEED > MAX_LINE_FOLLOW_SPEED:
            DRIVE_SPEED = MAX_LINE_FOLLOW_SPEED
        wait(RAMP_UP_DELAY_LF_MS)

def manage_speed_down(current_distance_traveled, target_distance):
    global DRIVE_SPEED
    distance_remaining = target_distance - current_distance_traveled
    if 0 < distance_remaining <= SLOW_DOWN_START_DISTANCE:
        progress = (SLOW_DOWN_START_DISTANCE - distance_remaining) / SLOW_DOWN_START_DISTANCE
        new_speed = MAX_LINE_FOLLOW_SPEED - (progress * (MAX_LINE_FOLLOW_SPEED - MIN_DRIVE_SPEED))
        DRIVE_SPEED = max(new_speed, MIN_DRIVE_SPEED)
    elif distance_remaining <= 0:
        DRIVE_SPEED = MIN_DRIVE_SPEED
    else:
        DRIVE_SPEED = MAX_LINE_FOLLOW_SPEED

#--------------------------------------------------------------
## Gyro Turning Precision Parameters
# -------------------------------------------------------------
KP_turn = 18    # Proportional gain (Adjust this first)
KI_turn = 0.02  # Integral gain (Adjust this last, keep small)
KD_turn = 3.5   # Derivative gain (Adjust this second)

# Integral Windup Prevention Limits for Turning
INTEGRAL_MAX_TURN = 100
INTEGRAL_MIN_TURN = -100
TURN_SPEED_DPS = 250
#--------------------------------------------------------------
## Gyro Turning Precision Functions
# -------------------------------------------------------------
def precise_gyro_turn_from_start(initial_heading, turn_angle):

    # Absolute target angle
    absolute_target_angle = initial_heading + turn_angle

    # PID variables
    integral_turn = 0
    last_error_turn = 0

    ev3.screen.clear()
    ev3.screen.print("Turning to:", absolute_target_angle)

    while True:
        # Current gyro reading
        current_angle = gyro_sensor.angle()
        error_turn = absolute_target_angle - current_angle

        # --- PID calculations ---
        P_turn = KP_turn * error_turn

        # Integral with anti-windup
        integral_turn += error_turn
        integral_turn = max(min(integral_turn, INTEGRAL_MAX_TURN), INTEGRAL_MIN_TURN)

        # Reset integral if error crosses zero
        if (error_turn > 0 and last_error_turn < 0) or (error_turn < 0 and last_error_turn > 0):
            integral_turn = 0

        I_turn = KI_turn * integral_turn

        # Derivative
        derivative_turn = error_turn - last_error_turn
        D_turn = KD_turn * derivative_turn

        # Total PID output → now called turn_speed for clarity
        turn_speed = P_turn + I_turn + D_turn
        turn_speed = max(min(turn_speed, TURN_SPEED_DPS), -TURN_SPEED_DPS)

        # Apply power for in-place rotation
        left_motor.run(turn_speed)
        right_motor.run(-turn_speed)

        # Stop condition: error small and motor speed low
        if abs(error_turn) < 1 and abs(turn_speed) < 5:
            break

        # Update last error
        last_error_turn = error_turn

        # Small delay for stability
        wait(10)

    # Stop motors and finalize
    left_motor.brake()
    right_motor.brake()
    wait(50)
    ev3.speaker.beep()
    ev3.screen.clear()
    ev3.screen.print("Turn Complete!")

# -------------------------------------------------------------
## Pivot Turning GRYO PID
# -------------------------------------------------------------
def precise_gyro_pivot_turn(initial_heading, turn_angle):
    # Set the absolute target angle based on the initial heading and the turn angle.
    absolute_target_angle = initial_heading + turn_angle

    integral_turn = 0
    last_error_turn = 0
    TURN_SPEED_DPS = 300  # You can adjust this speed for the pivot turn

    ev3.screen.print("Pivoting to:", absolute_target_angle)

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

def precise_gyro_backward_pivot_turn(initial_heading, turn_angle):
    absolute_target_angle = initial_heading + turn_angle
    print(absolute_target_angle)
    integral_turn = 0
    last_error_turn = 0
    TURN_SPEED_DPS = 300

    ev3.screen.print("Backward pivoting to:", absolute_target_angle)

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
#----------------------------------------------------------------------#
# --- Front Arm Control---#
#----------------------------------------------------------------------#
def front_arm_down(speed=200):
    Front_ARM.run_target(speed, FrontArm_DOWN_POSITION, then=Stop.HOLD, wait=True)
    ev3.speaker.beep()

def front_arm_up(speed=200):
    Front_ARM.run_target(speed, FrontArm_UP_POSITION, then=Stop.HOLD, wait=True)
    ev3.speaker.beep()
#----------------------------------------------------------------------#
# --- Back Arm Control ---#
#----------------------------------------------------------------------#
def go_grab_down(speed=200):
    Back_ARM.run_target(speed, BackArm_DOWN_POSITION_MOTOR_ANGLE, then=Stop.HOLD, wait=True)
    ev3.speaker.beep()

def go_grab_up(speed=200):
    Back_ARM.run_target(speed, BackArm_UP_POSITION_MOTOR_ANGLE, then=Stop.HOLD, wait=True)
    ev3.speaker.beep()

#----------------------------------------------------------------------#
# --- Finding Line Back---#
#----------------------------------------------------------------------#
def forward_until_black():
    ev3.speaker.beep()
    ev3.screen.print("Line finding")
    robot.drive(100, 0)
    while not (
        line_left.reflection() < BLACK_CALIBRATED_VALUE and
        line_right.reflection() < BLACK_CALIBRATED_VALUE):
        wait(10)
    robot.stop()
    ev3.speaker.beep()
    ev3.screen.print("Line Found!")
    wait(25)

def backward_until_black():
    ev3.speaker.beep()
    ev3.screen.print("Line finding (backward)")
    robot.drive(-100, 0)   # negative speed = backward
    while not (
        line_left.reflection() < BLACK_CALIBRATED_VALUE and
        line_right.reflection() < BLACK_CALIBRATED_VALUE):
        wait(10)
    robot.stop()
    ev3.speaker.beep()
    ev3.screen.print("Line Found (backward)!")
    wait(25)

    
#----------------------------------------------------------------------#
# State: Line Following to Distance ---# STATE_LINE_FOLLOWING_TO_DISTANCE
#----------------------------------------------------------------------#
def line_follow_to_distance(target_distance, next_state=STATE_MISSION_BARRIER_COLOR, 
                            reset_distance=True, tolerance=10):

    global current_state, integral, last_error, DRIVE_SPEED, state_initialization_done

    # Initialize state once
    if not state_initialization_done:
        if reset_distance:
            robot.reset()
        integral = 0
        last_error = 0
        DRIVE_SPEED = INITIAL_LINE_FOLLOW_SPEED
        ev3.screen.print("Line follow to:", target_distance, "mm")
        state_initialization_done = True

    # Sensor readings
    reflection_left = line_left.reflection()
    reflection_right = line_right.reflection()
    current_distance = robot.distance()

    # PID error calculation
    error = reflection_left - reflection_right
    P_term = KP * error
    integral = max(min(integral + error, INTEGRAL_MAX), INTEGRAL_MIN)
    I_term = KI * integral
    derivative = error - last_error
    D_term = KD * derivative

    turn_rate = P_term + I_term + D_term
    turn_rate = max(min(turn_rate, MAX_STEERING_RATE), MIN_STEERING_RATE)

    # Manage speed depending on distance to target
    if current_distance < (target_distance - SLOW_DOWN_START_DISTANCE):
        manage_speed_up()
    else:
        manage_speed_down(current_distance, target_distance)

    # Drive robot
    robot.drive(DRIVE_SPEED, turn_rate)
    last_error = error

    # Check stop condition with tolerance band
    if current_distance >= target_distance - tolerance:
        # Soft stop instead of hard brake
        robot.stop(Stop.COAST)
        ev3.speaker.beep()
        ev3.screen.print("\nTarget Distance Reached!")
        state_initialization_done = False  # Allow reuse with new distance

        # Switch state if provided
        if next_state is not None:
            current_state = STATE_MISSION_BARRIER_COLOR

def line_follow_after_black_block(target_distance,  
                                  reset_distance=True, tolerance=10):

    global current_state, integral, last_error, DRIVE_SPEED, state_initialization_done

    # Initialize state once
    if not state_initialization_done:
        if reset_distance:
            robot.reset()
        integral = 0
        last_error = 0
        DRIVE_SPEED = INITIAL_LINE_FOLLOW_SPEED
        ev3.screen.print("Line follow (after black block) to:", target_distance, "mm")
        state_initialization_done = True

    # Sensor readings
    reflection_left = line_left.reflection()
    reflection_right = line_right.reflection()
    current_distance = robot.distance()

    # PID error calculation
    error = reflection_left - reflection_right
    P_term = KP * error
    integral = max(min(integral + error, INTEGRAL_MAX), INTEGRAL_MIN)
    I_term = KI * integral
    derivative = error - last_error
    D_term = KD * derivative

    turn_rate = P_term + I_term + D_term
    turn_rate = max(min(turn_rate, MAX_STEERING_RATE), MIN_STEERING_RATE)

    # Manage speed depending on distance to target
    if current_distance < (target_distance - SLOW_DOWN_START_DISTANCE):
        manage_speed_up()
    else:
        manage_speed_down(current_distance, target_distance)

    # Drive robot
    robot.drive(DRIVE_SPEED, turn_rate)
    last_error = error

    # Check stop condition with tolerance band
    if current_distance >= target_distance - tolerance:
        # Soft stop instead of hard brake
        robot.stop(Stop.COAST)
        ev3.speaker.beep()
        ev3.screen.print("\nTarget Distance Reached (after black block)!")
        state_initialization_done = False  # Allow reuse with new distance

        # Switch state if provided
        
    

#----------------------------------------------------------------------#
# --- Mission Placing two Red and Yellow Bars---# STATE_MISSION_BARRIER_COLOR
#----------------------------------------------------------------------#
def mission_barrier_color():
    global current_state, initial_heading
    robot.stop()
    robot.straight(100)
    front_arm_down(200)
    robot.stop()
    precise_gyro_turn_from_start(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    
    #yellow
    robot.straight(-390)
    robot.stop()
    precise_gyro_turn_from_start(initial_heading, -50)
    initial_heading = gyro_sensor.angle()
    robot.straight(-110)
    front_arm_up(200)

    #red_pickup
    robot.straight(-50)
    front_arm_down(200)
    robot.straight(-60)

    #red
    robot.stop()
    precise_gyro_turn_from_start(initial_heading, -80)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(120)
    front_arm_up(200)
    robot.stop()
    ev3.speaker.beep()

    current_state = STATE_BARRIER_RED_SLIDING
    #current_state = STATE_STOP
#-----------------------------------------------------------------------------------------------#
#-----------red_bar_pulling----#STATE_BARRIER_RED------------------------------------------------------#
#-----------------------------------------------------------------------------------------------#
def redBar_Slide():
    global current_state, initial_heading
    robot.settings(stright_speed=500) ## setting the straight speed to 500
    robot.straight(-450)
    robot.stop()
    precise_gyro_turn_from_start(initial_heading, 130)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(-200)
    go_grab_down()
    robot.stop()
    robot.straight(250)
    robot.stop()
    go_grab_up()
    robot.straight(100)
    forward_until_black()
    robot.stop()
    precise_gyro_turn_from_start(initial_heading, 100)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    forward_until_black()
    ev3.speaker.beep()
    current_state = STATE_COLOR_SENSING
    #current_state = STATE_STOP
#-----------------------------------------------------------------------------------------------#
#-----------Color Sensing State------------------------------------------------------#
#-----------------------------------------------------------------------------------------------#
def go_to_scan_black_block_color():
    global current_state, state_initialization_done, initial_heading, detected_colors,detected_color_Square   
    print("Going to scan black block color...")
    robot.straight(200)
    robot.stop()

    precise_gyro_turn_from_start(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(300)
    robot.stop()
    precise_gyro_turn_from_start(initial_heading, -30)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    ### Sensing color for Black Square Block
    wait(200)
    detected_color_Square  = get_color(color_sensor)
    ev3.speaker.say ("Detected: {}".format(detected_color_Square))
    wait(200) 
    go_to_black_block()
    current_state = STATE_LINE_FOLLOW_AFTER_BLACK_BLOCK
    #current_state = STATE_STOP
# ----------------------------------------------------------------------#
########  Function before return to line
# ----------------------------------------------------------------------#
def go_to_black_block():
    global current_state, initial_heading, state_initialization_done
    precise_gyro_turn_from_start(initial_heading, 30)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    print("Going to black block line...")
    ### Bumping to reset angles
    robot.straight(-500) 
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    initial_heading = gyro_sensor.angle()
    ev3.speaker.beep()
  ###########################
    forward_until_black()
    robot.stop()
    robot.straight(60)
    robot.stop()

    precise_gyro_turn_from_start(initial_heading, -90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    state_initialization_done = False
# ----------------------------------------------------------------------#
########  Functions for Mission Taking Black Block
# ----------------------------------------------------------------------#
def bump_black_block():
    global current_state, initial_heading
    precise_gyro_turn_from_start(initial_heading, 180)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    ### Bumping to reset angles
    robot.straight(-300) 
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    initial_heading = gyro_sensor.angle()
    ev3.speaker.beep()

# ----------------------------------------------------------------------#
########  State Mission Taking Black Block 
# ----------------------------------------------------------------------#

def Taking_Color_Black_Block(): 
    global current_state, detected_color_Square,initial_heading
    bump_black_block()  
    if detected_color_Square == "Red":
        red_state()
    elif detected_color_Square == "Blue":
        blue_state()
    elif detected_color_Square == "Yellow":
        yellow_state()
    else:
        green_state()

    current_state = STATE_PUSHING_FLAG
    #current_state = STATE_STOP

# ----------------------------------------------------------------------#
def yellow_state():
    global current_state, initial_heading

    robot.straight(550)
    robot.stop()

    precise_gyro_turn_from_start(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(450)
    robot.stop()

    precise_gyro_turn_from_start(initial_heading, -90)
    initial_heading = gyro_sensor.angle()
    robot.stop()

    robot.straight(200)
    robot.stop()
    
    precise_gyro_turn_from_start(initial_heading, -90)
    initial_heading = gyro_sensor.angle()
    robot.stop()

    robot.straight(100)
    robot.stop()

    front_arm_down()
    robot.stop()

    robot.straight(-130)
    
    robot.stop()

    precise_gyro_turn_from_start(initial_heading, -90)
    initial_heading = gyro_sensor.angle()
    robot.stop()

    robot.straight(495)
    robot.stop()

    front_arm_up()
    robot.stop()

    robot.straight(-100)
    robot.stop()

    

# ----------------------------------------------------------------------#
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# ~~~~~~~ Take The BLUE COLOR AND PLACE ~~~~~~~~#
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
def blue_state():
    global current_state, initial_heading

    #front_arm_up()
    robot.stop()

    robot.straight(100)
    robot.stop()

    precise_gyro_turn_from_start(initial_heading, -90)
    initial_heading = gyro_sensor.angle()
    robot.stop()

    robot.straight(152.5)
    robot.stop()

    front_arm_down()
    robot.stop()

    robot.straight(160)
    robot.stop()

    precise_gyro_turn_from_start(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()

    robot.straight(495)
    robot.stop()

    front_arm_up()
    robot.stop()

    robot.straight(-100)
    robot.stop()
    
# ----------------------------------------------------------------------#
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# ~~~~~~~ Take The GREEN COLOR AND PLACE ~~~~~~~~#
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
def green_state():
    global current_state,initial_heading

    robot.straight(100)
    robot.stop()

    precise_gyro_turn_from_start(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()

    robot.straight(200)
    robot.stop()

    precise_gyro_turn_from_start(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()

    go_grab_down(200)

    robot.straight(200)
    robot.stop()

    precise_gyro_turn_from_start(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()

    robot.straight(100)
    robot.stop()
    go_grab_up(200)

    robot.straight(100)
    robot.stop()

# ----------------------------------------------------------------------#
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# ~~~~~~~ Take The RED COLOR AND PLACE ~~~~~~~~#
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
def red_state():
    global current_state, initial_heading

    robot.straight(550)
    robot.stop()

    precise_gyro_turn_from_start(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()

    robot.straight(250)
    robot.stop()

    precise_gyro_turn_from_start(initial_heading, 180)
    initial_heading = gyro_sensor.angle()
    robot.stop()

    robot.straight(250)
    robot.stop()

    front_arm_down()

    precise_gyro_turn_from_start(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()

    robot.straight(200)
    robot.stop()

    precise_gyro_turn_from_start(initial_heading, -90)
    initial_heading = gyro_sensor.angle()
    robot.stop()

    robot.straight(495)
    robot.stop()

    front_arm_up()

    robot.straight(-100)
    robot.stop()

#----------------------------------------------------------------------#
# --- Mission Pushing The Flag
#----------------------------------------------------------------------#
def Pushing_Flag():
    global state_initialization_done, current_state,initial_heading
    robot.straight(-220)
    robot.stop()
    front_arm_up()
    precise_gyro_turn_from_start(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(-120)
    robot.stop()
    go_grab_down()
    wait(200)
    go_grab_up()
    """Performs a sequence of turns and movements to a drop-off location."""
    precise_gyro_turn_from_start(initial_heading, 80)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(150)
    robot.stop()
    precise_gyro_turn_from_start(initial_heading, -80)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(-150)
    robot.stop()
    """Drops the object and returns to the starting position."""
    Back_ARM.run_target(speed=500, target_angle=100, then=Stop.HOLD, wait=True)
    wait(200)
    robot.straight(150)
    wait(200)
    robot.straight(-100)
    Back_ARM.run_target(speed=500, target_angle=0, then=Stop.HOLD, wait=True)
    robot.straight(200)
    state_initialization_done = False
    current_state = STATE_PAYLOAD_TAKING
    #current_state = STATE_STOP
#----------------------------------------------------------------------#
# ---Mission First Pay Load Taking
#----------------------------------------------------------------------#
def Payload_Taking():
    global state_initialization_done, current_state,initial_heading
    robot.straight(-80)
    robot.stop()
    precise_gyro_turn_from_start(initial_heading, 100)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    ### Bumping to reset angles
    robot.straight(500) 
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    initial_heading = gyro_sensor.angle()
    ev3.speaker.beep()
    robot.stop()
    #################
    precise_gyro_turn_from_start(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(250)
    robot.stop()
    front_arm_down()
    robot.straight(-150)
    precise_gyro_turn_from_start(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    ### Bumping to reset angles
    robot.straight(500) 
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    initial_heading = gyro_sensor.angle()
    ev3.speaker.beep()
    robot.stop()
    #################
    state_initialization_done = False
    current_state = STATE_FIRST_PAYLOAD_PLACING
    #current_state = STATE_STOP

#----------------------------------------------------------------------#
# ---Mission First Pay Load placing---#
#----------------------------------------------------------------------#
def First_Payload_Placing():
    global state_initialization_done, current_state,intial_heading
    robot.straight(300)
    precise_gyro_turn_from_start(initial_heading, -90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(500)
    robot.stop()
    precise_gyro_turn_from_start(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(20)
    robot.stop()
    front_arm_up()
    precise_gyro_turn_from_start(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    front_arm_down()
    robot.straight(300)
    robot.stop()
    precise_gyro_turn_from_start(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
     ### Bumping to reset angles
    robot.straight(-500) 
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    initial_heading = gyro_sensor.angle()
    ev3.speaker.beep()
    robot.stop()
    #################
    state_initialization_done = False
    current_state = STATE_SECOND_PICKING
    #current_state = STATE_STOP
#----------------------------------------------------------------------#
# ---Mission Second Pay Load taking---#
#----------------------------------------------------------------------#
def Second_Picking():
    global state_initialization_done, current_state,initial_heading
    precise_gyro_turn_from_start(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(100)
    robot.stop()
    front_arm_up()
    robot.straight(100)
    robot.stop()
    front_arm_down()
    robot.straight(-200)
    robot.stop()
    precise_gyro_turn_from_start(initial_heading, 180)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(300)
    state_initialization_done = False
    current_state = STATE_LINE_FOLLOW_TO_BARRIER_PUSH
    #current_state = STATE_STOP

#----------------------------------------------------------------------#
# ---Mission Second Pay Load placing---#
#----------------------------------------------------------------------#
def Payload_Final():
    global state_initialization_done, current_state,initial_heading
    precise_gyro_turn_from_start(initial_heading, 180)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(700)
    robot.stop()
    precise_gyro_turn_from_start(initial_heading, 90)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(20)
    robot.stop()
    front_arm_up()
    robot.straight(-20)
    robot.stop()
    state_initialization_done = False
    current_state = STATE_STOP


#---------------------------------------------------------------------#
# --- Barrier Pushing --- # STATE_BARRIER_SLIDING = 7
#---------------------------------------------------------------------#
def barrier_push():
    global state_initialization_done, current_state,initial_heading
    robot.straight(730)
    robot.stop()
    precise_gyro_turn_from_start(initial_heading, 180)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(233)
    robot.stop()
    precise_gyro_turn_from_start(initial_heading, 180)
    initial_heading = gyro_sensor.angle()
    robot.stop()
    robot.straight(200)
    robot.stop()
    state_initialization_done = False
    current_state = STATE_SECOND_PLACING

#----------------------------------------------------------------------#