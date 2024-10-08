import RPi.GPIO as GPIO
import time

# Motor GPIO pin setup
IN1 = 17  # GPIO 17
IN2 = 27  # GPIO 27
ENA = 12  # GPIO 12 (Enable pin for motor 1)
IN3 = 22  # GPIO 22
IN4 = 24  # GPIO 24
ENB = 18  # GPIO 18 (Enable pin for motor 2)

# Ultrasonic sensor GPIO pin setup
TRIG = 23  # Trigger pin
ECHO = 25  # Echo pin

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
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

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

# Function to measure distance using the ultrasonic sensor
def measure_distance():
    # Set Trigger to HIGH
    GPIO.output(TRIG, True)
    # Set Trigger to LOW after 10 microseconds
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # Measure the time for the Echo pin to go HIGH
    while GPIO.input(ECHO) == 0:
        start_time = time.time()

    while GPIO.input(ECHO) == 1:
        end_time = time.time()

    # Calculate the distance based on the speed of sound
    elapsed_time = end_time - start_time
    distance = (elapsed_time * 34300) / 2  # Speed of sound is 34300 cm/s

    return distance

try:
    # Start the motors driving forward at 50% speed
    drive_forward(100)
    print("Motors driving forward at 50% speed")

    while True:
        # Measure distance
        dist = measure_distance()
        print(f"Distance: {dist:.2f} cm")

        # Check if distance is less than 10 cm
        if dist < 10:
            print("Obstacle detected! Stopping motors.")
            stop_motors()
            break

        time.sleep(0.1)  # Small delay before measuring again

except KeyboardInterrupt:
    print("Measurement stopped by user")

finally:
    # Stop PWM and clean up GPIO settings
    pwm1.stop()
    pwm2.stop()
    GPIO.cleanup()
