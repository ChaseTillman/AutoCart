from rpi_hardware_pwm import HardwarePWM
from gpiozero import OutputDevice, DistanceSensor
import pygame
import time

# Initialize Pygame for keyboard event handling
pygame.init()
screen = pygame.display.set_mode((100, 100))

# Initialize PWM objects for speed control
pwm_right_motor = HardwarePWM(chip=2, pwm_channel=0, hz=20000)  # Right motor on channel 0
pwm_left_motor = HardwarePWM(chip=2, pwm_channel=1, hz=20000)  # Left motor on channel 1

# GPIO pins for direction control, forward is 'off'
right_motor_direction = OutputDevice(17, initial_value=False)  # GPIO 17 for right motor direction
left_motor_direction = OutputDevice(18, initial_value=False)  # GPIO 18 for left motor direction

# Initialize the ultrasonic sensor
sensor = DistanceSensor(echo=23, trigger=24)

# Target distance from the wall (in meters)
target_distance = 1.0

# PID coefficients
Kp = 1.0  # Proportional gain
Ki = 0.1  # Integral gain
Kd = 0.05  # Derivative gain

# Initial PID variables
integral = 0
last_error = 0

# Base speed for the left motor
left_motor_base_speed = 35  # Constant speed for the left motor
right_motor_base_speed = 60

# Function to control motor speeds
def control_motors(left_speed, right_speed):
    left_motor_direction.off()  # Set forward direction
    right_motor_direction.off()
    pwm_left_motor.change_duty_cycle(left_speed)
    pwm_right_motor.change_duty_cycle(right_speed)

# Function to update right motor speed using PID control
def update_speed(current_distance):
    global integral, last_error

    # Calculate the error
    error = target_distance - current_distance

    # Integral term calculation
    integral += error * 0.1  # assume dt = 0.1 seconds

    # Derivative term calculation
    derivative = (error - last_error) / 0.1  # dt = 0.1 seconds

    # Update last error
    last_error = error

    # PID output
    output = Kp * error + Ki * integral + Kd * derivative

    # Calculate new right motor speed
    right_motor_speed = max(min(right_motor_base_speed + output, 100), 0)

    return left_motor_base_speed, right_motor_speed

def main():
    pwm_left_motor.start(0)
    pwm_right_motor.start(0)

    try:
        while True:
            current_distance = sensor.distance
            left_speed, right_speed = update_speed(current_distance)
            control_motors(left_speed, right_speed)

            # Check for emergency stop
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    pwm_left_motor.stop()
                    pwm_right_motor.stop()
                    left_motor_direction.close()
                    right_motor_direction.close()
                    pygame.quit()
                    return  # Exit the script immediately

            time.sleep(0.1)

    finally:
        pwm_left_motor.stop()
        pwm_right_motor.stop()
        left_motor_direction.close()
        right_motor_direction.close()
        pygame.quit()

if __name__ == "__main__":
    main()
