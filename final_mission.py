import socket
import RPi.GPIO as GPIO
import time
from pid_controller import CustomPID  # Import your custom PID controller

# Define motor control pins
ENA, ENB, ENC, MOTOR_RIGHT_FORWARD, MOTOR_RIGHT_BACKWARD = 7, 21, 22, 1, 16
MOTOR_LEFT_FORWARD, MOTOR_LEFT_BACKWARD, COLLECTION_MOTOR_1, COLLECTION_MOTOR_2 = 27, 17, 12, 20

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

# PWM setup
pwm_enable_right = GPIO.PWM(ENA, 1000)
pwm_enable_left = GPIO.PWM(ENB, 1000)
pwm_collection = GPIO.PWM(ENC, 1000)
pwm_enable_left.start(100)
pwm_enable_right.start(100)

# PID controller setup
pid = CustomPID(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=0)
pid.output_limits = (-70, 70)  # Limit adjustments to -70% to +70%

# Define waypoints for the pattern
waypoints = [
    (0, 1.4), (0.4, 1.4), (0.4, -0.2), (0.8, -0.2), (0.8, 1.4), (1.2, 1.4),
    (1.2, -0.2), (1.6, -0.2), (1.6, 1.4), (1.4, 1.4), (1.4, 0.4), (1, 0.4),
    (1, 1.2), (0.6, 1.2), (0.6, 0.4), (0.2, 0.4), (0.2, 1.2), (0, 1.2), (0, 0)
]
current_waypoint_index = 0

# Other control variables
snake_sign_active = False
snake_sign_position = None

# Set up TCP client to receive positional data
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('192.168.2.1', 9001))  # IP of MacBook

# Helper functions for motor control
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

# Adjust the driving to stay straight using PID
def adjust_for_straightness(delta_x):
    correction = pid.update(delta_x)
    left_pwm = max(70, min(100, 100 - correction))
    right_pwm = max(70, min(100, 100 + correction))
    pwm_enable_left.ChangeDutyCycle(left_pwm)
    pwm_enable_right.ChangeDutyCycle(right_pwm)

# Helper function to reverse to the saved position
def reverse_to_position(saved_position):
    saved_x, saved_y = saved_position
    while True:
        # Get the current position (assuming you have access to `delta_x` and `delta_y`)
        data = client_socket.recv(1024).decode()
        delta_x, delta_y, delta_z = map(float, data.split(","))
        
        # Drive backward if not at the saved position
        if abs(delta_x - saved_x) > 0.05 or abs(delta_y - saved_y) > 0.05:
            # Reverse logic (drive backward)
            GPIO.output(MOTOR_RIGHT_BACKWARD, GPIO.HIGH)
            GPIO.output(MOTOR_LEFT_BACKWARD, GPIO.HIGH)
            pwm_enable_left.ChangeDutyCycle(100)
            pwm_enable_right.ChangeDutyCycle(100)
        else:
            # Stop when reached the saved position
            stop_motors()
            print(f"Returned to saved position: ({saved_x}, {saved_y})")
            break

# Deposit sequence
def deposit_sequence():
    # Drive forward for 2 seconds
    drive_forward()
    time.sleep(2)
    stop_motors()

    # Stop for 10 seconds
    print("Waiting for 10 seconds...")
    time.sleep(10)

    # Reverse for 2 seconds
    GPIO.output(MOTOR_RIGHT_BACKWARD, GPIO.HIGH)
    GPIO.output(MOTOR_LEFT_BACKWARD, GPIO.HIGH)
    pwm_enable_left.ChangeDutyCycle(100)
    pwm_enable_right.ChangeDutyCycle(100)
    time.sleep(2)
    stop_motors()
    print("Deposit complete.")

# Main mission loop
try:
    while True:
        # Receive positional data from MacBook
        data = client_socket.recv(1024).decode()
        if not data:
            break
        delta_x, delta_y, delta_z = map(float, data.split(","))

        # Adjust to keep straight
        adjust_for_straightness(delta_x)

        # Check if snake sign is active
        if snake_sign_active:
            # Handle snake sign: drive straight until the boundary is reached
            drive_forward()
            if delta_x < -0.1 or delta_x > 1.7 or delta_y < -0.3 or delta_y > 1.5:
                stop_motors()
                print(f"Boundary reached at ({delta_x}, {delta_y}). Stopping for 30 seconds.")
                time.sleep(30)  # Stop for 30 seconds
                print("Reversing back to the saved position.")
                reverse_to_position(snake_sign_position)  # Reverse to saved position
                snake_sign_active = False  # Deactivate snake sign
            continue  # Skip waypoint navigation during snake sign

        # Save the current position when the snake sign is first activated
        if data == 's':  # Snake sign command received
            snake_sign_active = True
            snake_sign_position = (delta_x, delta_y)
            print(f"Snake sign activated. Saving current position: {snake_sign_position}")

        # Get current waypoint
        target_x, target_y = waypoints[current_waypoint_index]

        # Logic to move towards the waypoint
        if abs(delta_x - target_x) > 0.05:  # Need to correct X-axis
            if delta_x < target_x:
                turn_right()
            else:
                turn_left()
            time.sleep(0.5)  # Adjust time to complete the turn
        elif abs(delta_y - target_y) > 0.05:  # Need to correct Y-axis
            drive_forward()

        # Check if we reached the current waypoint
        if abs(delta_x - target_x) < 0.05 and abs(delta_y - target_y) < 0.05:
            current_waypoint_index += 1
            if current_waypoint_index >= len(waypoints):
                print("Mission complete. Waiting for deposit command.")
                stop_motors()  # Stop motors after mission completion
                break  # Mission complete, stop here

    # After mission completion, wait for 'd' key to trigger deposit
    while True:
        data = client_socket.recv(1024).decode()  # Waiting for deposit signal ('d')
        if data == 'd':  # 'd' key pressed to start deposit
            print("Deposit signal received.")
            deposit_sequence()
            break

except KeyboardInterrupt:
    print("Mission interrupted.")

finally:
    # Clean up GPIO and close socket
    client_socket.close()
    GPIO.cleanup()
