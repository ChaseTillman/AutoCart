from rpi_hardware_pwm import HardwarePWM
import time
# Assuming chip2 exists and has at least two channels: 0 and 1
pwm1 = HardwarePWM(pwm_channel=0, hz=100, chip=2)
pwm2 = HardwarePWM(pwm_channel=1, hz=100, chip=2)

# Example configuration
pwm_frequency = 20000  # 20 kHz
duty_cycle = 50  # 50%

# Initialize PWM signals
pwm1.start(100)
pwm2.start(100)

# Do something with the PWM signals
pwm1.change_duty_cycle(60)

pwm1.change_frequency(20000)

pwm2.change_duty_cycle(40)

pwm2.change_frequency(20000)

try:
    while True:
        time.sleep(1)

except KeyboardInterrupt:
# Cleanup
    pwm1.stop()
    pwm2.stop()
