import RPi.GPIO as GPIO
import time

# Set up GPIO mode to use Broadcom numbering (BCM)
GPIO.setmode(GPIO.BCM)

# Define GPIO pins for motor 1
IN1 = 17  # GPIO 17
IN2 = 18  # GPIO 18
ENA = 12  # GPIO 12 (Enable pin for motor 1)

# Define GPIO pins for motor 2
IN3 = 22  # GPIO 22
IN4 = 23  # GPIO 23
ENB = 13  # GPIO 13 (Enable pin for motor 2)

# Set up GPIO pins as outputs
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# Set up PWM for speed control
pwm1 = GPIO.PWM(ENA, 1000)  # Set PWM frequency to 1 kHz
pwm2 = GPIO.PWM(ENB, 1000)  # Set PWM frequency to 1 kHz
pwm1.start(100)  # Start PWM with 100% duty cycle
pwm2.start(100)  # Start PWM with 100% duty cycle

# Function to drive motor 1 forward
def drive_motor1_forward():
    print("Driving motor 1 forward")
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)

# Function to drive motor 1 backward
def drive_motor1_backward():
    print("Driving motor 1 backward")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)

# Function to drive motor 2 forward
def drive_motor2_forward():
    print("Driving motor 2 forward")
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

# Function to drive motor 2 backward
def drive_motor2_backward():
    print("Driving motor 2 backward")
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

try:
    # Test motor 1 forward and backward
    drive_motor1_forward()
    time.sleep(5)
    drive_motor1_backward()
    time.sleep(5)
    
    # Test motor 2 forward and backward
    drive_motor2_forward()
    time.sleep(5)
    drive_motor2_backward()
    time.sleep(5)

finally:
    # Stop PWM and clean up GPIO settings
    pwm1.stop()
    pwm2.stop()
    GPIO.cleanup()
