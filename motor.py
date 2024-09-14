import RPi.GPIO as GPIO
import time

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)

# Define GPIO pins
ENA = 32  # Enable pin for Motor 1
IN1 = 17
IN2 = 18
ENB = 33  # Enable pin for Motor 2
IN3 = 22
IN4 = 23

# Set up GPIO pins
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

# Set up PWM for speed control (optional)
pwm1 = GPIO.PWM(EN1, 1000)  # 1kHz frequency
pwm2 = GPIO.PWM(EN2, 1000)

# Start PWM with 100% duty cycle
pwm1.start(100)
pwm2.start(100)

# Function to drive motor forward
def drive_forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

# Function to drive motor backward
def drive_backward():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

# Run motors forward for 5 seconds
drive_forward()
time.sleep(5)

# Stop motors
pwm1.stop()
pwm2.stop()

# Clean up GPIO settings
GPIO.cleanup()
