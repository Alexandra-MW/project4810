import RPi.GPIO as GPIO
from collections import deque
import time

# Motor GPIO pin setup
IN1 = 17  # GPIO 17
IN2 = 27  # GPIO 27
ENA = 12  # GPIO 12 (Enable pin for motor 1)
IN3 = 22  # GPIO 22
IN4 = 24  # GPIO 24
ENB = 18  # GPIO 18 (Enable pin for motor 2)

# Ultrasonic sensor GPIO pin setup
TRIG1 = 23  # Trigger pin
ECHO1 = 25  # Echo pin
TRIG2 = 16  # Trigger pin
ECHO2 = 20  # Echo pin

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)

# Set up motor pins as outputs
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# Set up ultrasonic sensor pins
GPIO.setup(TRIG1, GPIO.OUT)
GPIO.setup(ECHO1, GPIO.IN)
GPIO.setup(TRIG2, GPIO.OUT)
GPIO.setup(ECHO2, GPIO.IN)

# Set up PWM for speed control
pwm1 = GPIO.PWM(ENA, 1000)  # Motor 1 PWM at 1 kHz
pwm2 = GPIO.PWM(ENB, 1000)  # Motor 2 PWM at 1 kHz
pwm1.start(0)  # Start PWM with 0% duty cycle
pwm2.start(0)  # Start PWM with 0% duty cycle

# Function to drive both motors forward at a specified speed
def drive_forward(speed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(speed)

# Function to stop both motors
def stop_motors():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm1.ChangeDutyCycle(0)
    pwm2.ChangeDutyCycle(0)

# Function to turn the rover 180 degrees
def turn_around(speed, duration):
    # To turn around, stop one motor while running the other
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(0)
    
    time.sleep(duration)  # Run this for a set duration to achieve 180-degree turn

# Function to measure distance using the ultrasonic sensor
def measure_distance1():
    # Set Trigger to HIGH
    GPIO.output(TRIG1, True)
    # Set Trigger to LOW after 10 microseconds
    time.sleep(0.00001)
    GPIO.output(TRIG1, False)

    # Measure the time for the Echo pin to go HIGH
    while GPIO.input(ECHO1) == 0:
        start_time = time.time()

    while GPIO.input(ECHO1) == 1:
        end_time = time.time()

    # Calculate the distance based on the speed of sound
    elapsed_time = end_time - start_time
    distance = (elapsed_time * 34300) / 2  # Speed of sound is 34300 cm/s

    return distance

# Function to measure distance using the ultrasonic sensor
def measure_distance2():
    # Set Trigger to HIGH
    GPIO.output(TRIG2, True)
    # Set Trigger to LOW after 10 microseconds
    time.sleep(0.00001)
    GPIO.output(TRIG2, False)

    # Measure the time for the Echo pin to go HIGH
    while GPIO.input(ECHO2) == 0:
        start_time = time.time()

    while GPIO.input(ECHO2) == 1:
        end_time = time.time()

    # Calculate the distance based on the speed of sound
    elapsed_time = end_time - start_time
    distance = (elapsed_time * 34300) / 2  # Speed of sound is 34300 cm/s

    return distance

try:
    desired_distance = 20
    kp = 2
    distance_readings = deque(maxlen=5)
    last_valid_distance = None
    max_change_threshold = 3
    
    # Start the motors driving forward at 50% speed
    drive_forward(100)
    print("Motors driving forward at 100% speed")

    # Drive forward until an obstacle is detected within 20 cm
    while True:
        distance1 = measure_distance1()
        distance2 = measure_distance2()
        print(f"Distance1: {distance1:.2f} cm")
        print(f"Distance2: {distance2:.2f} cm")

        # Check if the new reading is within the allowed change threshold
        if last_valid_distance is None or abs(distance2 - last_valid_distance) <= max_change_threshold:
            distance_readings.append(distance2)
            last_valid_distance = distance2  # Update the last valid reading
        else:
            print(f"Ignored outlier reading: {distance2:.2f} cm")

        average_distance = sum(distance_readings)/len(distance_readings)
        wall_error = average_distance - desired_distance
        adjustment = kp * wall_error
    
        # Adjust motor speeds within the high range, ensuring they remain at or near 100%
        right_motor_speed = 100 - min(max(adjustment, -20), 20)  # Adjust between 80% to 100%
        left_motor_speed = 100 + min(max(adjustment, -20), 20)
            
        # Ensure PWM values are within 0 to 100 range
        right_motor_speed = max(0, min(100, right_motor_speed))
        left_motor_speed = max(0, min(100, left_motor_speed))
    
        # Set the PWM duty cycle to adjust motor speeds
        pwm1.ChangeDutyCycle(right_motor_speed)
        pwm2.ChangeDutyCycle(left_motor_speed)
        
        if distance1 < 20:
            print("Obstacle detected! Stopping motors.")
            stop_motors()
            break
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Operation stopped by user")

finally:
    # Stop PWM and clean up GPIO settings
    pwm1.stop()
    pwm2.stop()
    GPIO.cleanup()
