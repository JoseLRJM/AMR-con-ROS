import os

import RPi.GPIO as GPIO
import time

# Pin Definitions
input_pin = 4  # BCM pin 18, BOARD pin 12

def main():
    prev_value = None

    # Pin Setup:
    GPIO.setmode(GPIO.BCM)  # BCM pin-numbering scheme from Raspberry Pi
    GPIO.setup(input_pin, GPIO.OUT)  # set pin as an input pin
    print("Starting demo now! Press CTRL+C to exit")
    try:
        while True:
            GPIO.output(input_pin, GPIO.HIGH)
            time.sleep(0.001)
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()
    
