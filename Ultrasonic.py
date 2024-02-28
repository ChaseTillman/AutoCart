from gpiozero import DistanceSensor
from time import sleep

# Define the GPIO pins for the trigger and echo
trigger_pin = 23  # Adjust to your trigger pin
echo_pin = 24    # Adjust to your echo pin

# Initialize the sensor
sensor = DistanceSensor(echo=echo_pin, trigger=trigger_pin)

try:
    while True:
        print("Distance: {:.2f} cm".format(sensor.distance * 100))
        sleep(1)
except KeyboardInterrupt:
    print("Measurement stopped by user")
