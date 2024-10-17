import socket
import RPi.GPIO as GPIO
import time

# Set up motor pins (replace with actual GPIO pin numbers)
MOTOR_LEFT_FORWARD = 17  # Left motor forward
MOTOR_LEFT_BACKWARD = 27  # Left motor backward
MOTOR_RIGHT_FORWARD = 22  # Right motor forward
MOTOR_RIGHT_BACKWARD = 24  # Right motor backward
COLLECTION_MOTOR = 5  # Collection wheel motor

# Set up GPIO mode and pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_LEFT_FORWARD, GPIO.OUT)
GPIO.setup(MOTOR_LEFT_BACKWARD, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_FORWARD, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_BACKWARD, GPIO.OUT)
GPIO.setup(COLLECTION_MOTOR, GPIO.OUT)

# PWM Setup for motors and collection wheel
pwm_left_forward = GPIO.PWM(MOTOR_LEFT_FORWARD, 1000)
pwm_right_forward = GPIO.PWM(MOTOR_RIGHT_FORWARD, 1000)
pwm_left_backward = GPIO.PWM(MOTOR_LEFT_BACKWARD, 1000)
pwm_right_backward = GPIO.PWM(MOTOR_RIGHT_BACKWARD, 1000)
pwm_collection = GPIO.PWM(COLLECTION_MOTOR, 1000)

# Start collection motor and drive motors at 100%
pwm_left_forward.start(100)
pwm_right_forward.start(100)
pwm_collection.start(100)

# Variables for rover control
current_run = 0
direction = "forward"
u_turn = False
snake_sign_active = False

# Tolerances for straight driving
STRAIGHT_TOLERANCE = 0.01  # 1 cm deviation
PWM_ADJUST_STEP = 5  # Adjust motors by 5% to correct course

# TCP client to receive positional data from the MacBook
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('192.168.2.1', 8080))  # IP of MacBook 

def drive_forward():
    pwm_left_forward.ChangeDutyCycle(100)
    pwm_right_forward.ChangeDutyCycle(100)

def stop_motors():
    pwm_left_forward.ChangeDutyCycle(0)
    pwm_right_forward.ChangeDutyCycle(0)

def turn_left():
    pwm_left_forward.ChangeDutyCycle(0)
    pwm_right_forward.ChangeDutyCycle(100)

def turn_right():
    pwm_left_forward.ChangeDutyCycle(100)
    pwm_right_forward.ChangeDutyCycle(0)

def u_turn_rover():
    global u_turn
    if u_turn:
        turn_left()
    else:
        turn_right()
    time.sleep(2)  # Adjust time for a full U-turn
    u_turn = not u_turn  # Alternate the direction for the next U-turn

def deposit_collection():
    pwm_collection.ChangeDutyCycle(0)

def adjust_for_straightness(delta_x):
    """ Adjusts motor speeds to keep rover driving straight based on x-axis data. """
    if abs(delta_x) > STRAIGHT_TOLERANCE:
        if delta_x > 0:  # Rover drifting to the right, slow left motor
            pwm_left_forward.ChangeDutyCycle(100 - PWM_ADJUST_STEP)
            pwm_right_forward.ChangeDutyCycle(100)
        else:  # Rover drifting to the left, slow right motor
            pwm_left_forward.ChangeDutyCycle(100)
            pwm_right_forward.ChangeDutyCycle(100 - PWM_ADJUST_STEP)
    else:
        # If within tolerance, set motors back to 100%
        pwm_left_forward.ChangeDutyCycle(100)
        pwm_right_forward.ChangeDutyCycle(100)

try:
    while True:
        # Receive positional data
        data = client_socket.recv(1024).decode()
        if not data:
            break
        delta_x, delta_y, delta_z = map(float, data.split(","))

        # Adjust the motors based on x-axis deviation to drive straight
        adjust_for_straightness(delta_x)

        # Drive forward/backward based on Z-axis data
        if direction == "forward":
            if delta_z >= 1.8:
                stop_motors()
                u_turn_rover()
                current_run += 1
                direction = "backward"
            else:
                drive_forward()
        elif direction == "backward":
            if delta_z <= 0:
                stop_motors()
                u_turn_rover()
                current_run += 1
                direction = "forward"
            else:
                drive_forward()

        # Snake sign logic (stop motors and collection motor for 30 seconds)
        if snake_sign_active and (delta_z >= 1.8 or delta_z <= 0):
            print("Snake sign active, stopping for 30 seconds.")
            stop_motors()
            pwm_collection.ChangeDutyCycle(0)  # Stop collection motor
            time.sleep(30)
            snake_sign_active = False
            pwm_collection.ChangeDutyCycle(100)  # Restart collection motor

        # Adjust position for next pass (X-axis)
        if delta_z <= 0 and direction == "backward":
            delta_x += (current_run * 0.1)  # Shift 10cm after each pass
            if delta_x >= 1.9:
                print("Mission complete. Returning to deposit zone.")
                # Turn 90 degrees and drive back to deposit zone
                turn_right()  # Adjust turn based on rover's initial orientation
                time.sleep(1)  # Adjust for turning 90 degrees
                drive_forward()
                time.sleep(2)  # Adjust time for driving 1.8 meters
                deposit_collection()
                break

        # Check for keyboard commands (e.g., 's' for snake, 'd' for deposit)
        if data == 's':  # Snake sign command received
            snake_sign_active = True
        elif data == 'd':  # Deposit sign command received
            deposit_collection()

except KeyboardInterrupt:
    print("Mission interrupted.")

finally:
    # Clean up GPIO and close socket connection
    client_socket.close()
    GPIO.cleanup()
