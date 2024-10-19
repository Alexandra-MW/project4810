import socket
import RPi.GPIO as GPIO
import time
from simple_pid import PID

# Define motor control pins 
ENA = 7   # Enable pin for right motor
ENB = 21  # Enable pin for left motor
ENC = 22  #Enable pin for collector 
MOTOR_RIGHT_FORWARD = 1  # Left motor forward
MOTOR_RIGHT_BACKWARD = 16  # Left motor backward
MOTOR_LEFT_FORWARD = 27  # Right motor forward
MOTOR_LEFT_BACKWARD = 17  # Right motor backward
COLLECTION_MOTOR_1 = 12  # Collection wheel motor IN1
COLLECTION_MOTOR_2 = 20 #Collection wheel motor IN2

# Set up GPIO mode and pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(ENC, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_FORWARD, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_BACKWARD, GPIO.OUT)
GPIO.setup(MOTOR_LEFT_FORWARD, GPIO.OUT)
GPIO.setup(MOTOR_LEFT_BACKWARD, GPIO.OUT)
GPIO.setup(COLLECTION_MOTOR_1, GPIO.OUT)
GPIO.setup(COLLECTION_MOTOR_2, GPIO.OUT)

# PWM setup for enable pins and collection motor
pwm_enable_right = GPIO.PWM(ENA, 1000)
pwm_enable_left = GPIO.PWM(ENB, 1000)
pwm_collection = GPIO.PWM(ENC, 1000)

# Start collection motor and drive motors at 100%
pwm_enable_left.start(100)
pwm_enable_right.start(100)
#pwm_collection.start(100) #Turned off for navigation testing 

# PID controller for straight driving
pid = PID(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=0)  # Adjust PID values as necessary
pid.output_limits = (-70, 70)  # Limit adjustments to -50% to +50%

# Mission control variables
current_run = 0
direction = "forward"
u_turn = False
snake_sign_active = False

# Tolerances for driving straight
STRAIGHT_TOLERANCE = 0.02  # 1 cm deviation

# Set up TCP client to receive positional data
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('192.168.2.1', 8080))  # IP of MacBook (server)

# Motor control functions
def drive_forward():
    GPIO.output(MOTOR_RIGHT_FORWARD, GPIO.HIGH)
    GPIO.output(MOTOR_LEFT_FORWARD, GPIO.HIGH)
    pwm_enable_left.ChangeDutyCycle(100)
    pwm_enable_right.ChangeDutyCycle(100)

def stop_motors():
    pwm_enable_left.ChangeDutyCycle(0)
    pwm_enable_right.ChangeDutyCycle(0)

def turn_left():
    GPIO.output(MOTOR_LEFT_FORWARD, GPIO.LOW)
    GPIO.output(MOTOR_RIGHT_FORWARD, GPIO.HIGH)
    pwm_enable_left.ChangeDutyCycle(0)
    pwm_enable_right.ChangeDutyCycle(100)

def turn_right():
    GPIO.output(MOTOR_LEFT_FORWARD, GPIO.HIGH)
    GPIO.output(MOTOR_RIGHT_FORWARD, GPIO.LOW)
    pwm_enable_left.ChangeDutyCycle(100)
    pwm_enable_right.ChangeDutyCycle(0)

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

def adjust_for_straightness(delta_z):
    """ Adjust motor speeds to keep the rover driving straight using PID """
    correction = pid(delta_z)  # Get correction from PID controller
    # Adjust motor speeds based on PID output
    pwm_enable_left.ChangeDutyCycle(100 - correction)
    pwm_enable_right.ChangeDutyCycle(100 + correction)

# Main mission loop
try:
    while True:
        # Receive positional data from MacBook
        data = client_socket.recv(1024).decode()
        if not data:
            break
        delta_x, delta_y, delta_z = map(float, data.split(","))

        # Adjust motors to keep the rover straight
        adjust_for_straightness(delta_z)

        # Driving logic based on Z-axis (forward/backward)
        if direction == "forward":
            if delta_x >= 0.5:  # Forward limit reached
                stop_motors()
                u_turn_rover()
                current_run += 1
                direction = "backward"
            else:
                drive_forward()
        elif direction == "backward":
            if delta_x <= 0:  # Backward limit reached
                stop_motors()
                u_turn_rover()
                current_run += 1
                direction = "forward"
            else:
                drive_forward()

        # Snake sign logic (stop motors and collection motor for 30 seconds)
        if snake_sign_active and (delta_x >= 0.5 or delta_x <= 0):
            stop_motors()
            pwm_collection.ChangeDutyCycle(0)  # Stop collection motor
            time.sleep(30)
            snake_sign_active = False
            pwm_collection.ChangeDutyCycle(100)  # Restart collection motor

        # Shift X-axis for the next pass after a U-turn
        if delta_x <= 0 and direction == "backward":
            delta_z += (current_run * 0.1)  # Shift 10 cm after each pass
            if delta_z >= 1:  # Complete 10 passes (back and forth)
                print("Mission complete. Returning to deposit zone.")
                turn_right()  # Example: adjust turn based on start orientation
                time.sleep(1)  # Adjust for turning 90 degrees
                drive_forward()
                time.sleep(2)  # Drive 1.8 meters back to deposit zone
                deposit_collection()
                break

        # Handle keyboard commands from MacBook (s = snake, d = deposit)
        if data == 's':  # Snake sign command received
            snake_sign_active = True
        elif data == 'd':  # Deposit sign command received
            deposit_collection()

except KeyboardInterrupt:
    print("Mission interrupted.")

finally:
    # Clean up GPIO and close socket
    client_socket.close()
    GPIO.cleanup()
