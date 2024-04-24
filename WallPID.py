from rpi_hardware_pwm import HardwarePWM
from gpiozero import OutputDevice, DistanceSensor
import pygame
import time

# Initialize Pygame for keyboard event handling
pygame.init()
screen = pygame.display.set_mode((100, 100))

# Initialize PWM objects for speed control
pwm_right_motor = HardwarePWM(pwm_channel=0, hz=20000)  # Right motor on channel 0
pwm_left_motor = HardwarePWM(pwm_channel=1, hz=20000)  # Left motor on channel 1

# GPIO pins for direction control, forward is 'off'
right_motor_direction = OutputDevice(17, initial_value=True)  # GPIO 17 for right motor direction
left_motor_direction = OutputDevice(27, initial_value=True)  # GPIO 18 for left motor direction

# Initialize the ultrasonic sensor
sensor = DistanceSensor(echo=23, trigger=24)

# Target distance from the wall (in meters)
target_distance = 0.35

post_turn_pid_duration = 10

# PID coefficients
Kp = 4.0  # Proportional gain
Ki = 0.1  # Integral gain
Kd = 0.2  # Derivative gain

# Initial PID variables
integral = 0
last_error = 0

# Base speed for the left motor
left_motor_base_speed = 35  # Constant speed for the left motor
left_motor_base_speed = 36  # Constant speed for the right motor
# Function to control motor speeds
def control_motors(left_speed, right_speed):
    left_motor_direction.on()  # Set forward direction
    right_motor_direction.on()
    pwm_left_motor.change_duty_cycle(left_speed)
    pwm_right_motor.change_duty_cycle(right_speed)

# Function to update right motor speed using PID control
def update_speed(current_distance):
    global integral, last_error
    
    if current_distance == 1.0:
        turn_right()
        return left_motor_base_speed, left_motor_base_speed  # Reset to base speed after turn
    
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

    # Calculate new right motor speed - invert the output to correct the direction
    right_motor_speed = max(min(right_motor_base_speed - output, 100), 20)
    left_motor_speed = max(min(left_motor_base_speed + output, 100), 0)
  
    return left_motor_speed, right_motor_speed

def turn_right():
    control_motors(70, 25)
    time.sleep(5)
    control_motors(35, 35)
    return

def main():
    pwm_left_motor.start(0)
    pwm_right_motor.start(0)
    
    turn_performed = False
    start_time = time.time()
    
    try:
        while True:
            current_distance = sensor.distance
            left_speed, right_speed = update_speed(current_distance)
            control_motors(left_speed, right_speed)

            if current_distance == 1.0 and not turn_performed:
                    turn_right()
                    turn_performed = True
                    start_time = time.time()  # Reset the start time after the turn
    
            if turn_performed and (time.time() - start_time > post_turn_pid_duration):
                print("PID control duration elapsed. Stopping the cart.")
                break  # Stop the cart after the PID control duration
        
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
