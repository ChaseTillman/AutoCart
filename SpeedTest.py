from rpi_hardware_pwm import HardwarePWM
from gpiozero import OutputDevice
from pynput import keyboard
import time

# Initialize PWM objects for speed control
pwm_left_motor = HardwarePWM(pwm_chip=2, pwm_channel=1, hz=20000)  # Channel 1 for left motor speed
pwm_right_motor = HardwarePWM(pwm_chip=2, pwm_channel=2, hz=20000)  # Channel 2 for right motor speed

# GPIO pins for direction control, change pin numbers as needed
left_motor_direction = OutputDevice(20)  # Assuming GPIO 20 for left motor direction
right_motor_direction = OutputDevice(21)  # Assuming GPIO 21 for right motor direction

# Define full speed and reduced speed for turning
full_speed = 50
reduced_speed = 30  # Reduced speed for the motor on the inside of the turn

def control_motors(left_speed, right_speed):
    if left_speed >= 0:
        left_motor_direction.on()  # Forward
    else:
        left_motor_direction.off()  # Reverse

    if right_speed >= 0:
        right_motor_direction.on()  # Forward
    else:
        right_motor_direction.off()  # Reverse

    # Apply absolute speed value for PWM
    pwm_left_motor.change_duty_cycle(abs(left_speed))
    pwm_right_motor.change_duty_cycle(abs(right_speed))

def on_press(key):
    try:
        if key.char == 'w':  # Forward
            control_motors(full_speed, full_speed)
            print("Moving Forward")
        elif key.char == 's':  # Backward
            control_motors(-full_speed, -full_speed)
            print("Moving Backward")
        elif key.char == 'a':  # Turn Left
            control_motors(reduced_speed, full_speed)
            print("Turning Left")
        elif key.char == 'd':  # Turn Right
            control_motors(full_speed, reduced_speed)
            print("Turning Right")
    except AttributeError:
        pass  # Handles special keys pressed; do nothing

def on_release(key):
    if key.char in ['w', 's', 'a', 'd']:  # Stop motors on key release
        control_motors(0, 0)
        print(f"Stopping {key.char}")

def main():
    # Start PWM with initial duty cycle at 0
    pwm_left_motor.start(0)
    pwm_right_motor.start(0)

    # Listener for keyboard events
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    try:
        while True:
            time.sleep(0.1)  # Short sleep to prevent high CPU usage
    except KeyboardInterrupt:
        pwm_left_motor.stop()
        pwm_right_motor.stop()
        left_motor_direction.close()
        right_motor_direction.close()
        print("Program exited cleanly")

if __name__ == "__main__":
    main()
