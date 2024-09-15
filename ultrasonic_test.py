import RPi.GPIO as GPIO
import time

# Define GPIO pins for the ultrasonic sensor
TRIG = 23  # Trigger pin (using GPIO 27)
ECHO = 24   # Echo pin (using GPIO 4)

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

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
    while True:
        dist = measure_distance()
        print(f"Distance: {dist:.2f} cm")
        time.sleep(1)  # Wait 1 second before measuring again

except KeyboardInterrupt:
    print("Measurement stopped by user")
    GPIO.cleanup()  # Reset GPIO settings
