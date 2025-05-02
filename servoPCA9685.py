from i2cpy import I2C
import time

class Servo:
    def __init__(self):
        self.i2c = I2C(driver="ch341")
        self.PCA9685_ADDRESS = 0x40  # I2C address of PCA9685
        self.initialize_pca9685()
        self.speed = 1  # Speed of servo motor (0-1)
    
    def initialize_pca9685(self):
        # PCA9685 Registers
        # Put PCA9685 into sleep mode to configure prescaler
        self.i2c.writeto(self.PCA9685_ADDRESS, [0x00, 0x10])
        # Set prescaler for 50 Hz PWM frequency (prescaler = 121 for 50 Hz)
        self.i2c.writeto(self.PCA9685_ADDRESS, [0xFE, 121])
        # Wake up PCA9685 and enable auto-increment
        self.i2c.writeto(self.PCA9685_ADDRESS, [0x00, 0x20])

    def set_pwm(self, channel, on, off):
        MOTOR_ON_L = 0x06 + 4 * channel
        MOTOR_OFF_L = 0x08 + 4 * channel

        self.i2c.writeto(self.PCA9685_ADDRESS, [MOTOR_ON_L, on & 0xFF, (on >> 8) & 0xFF])
        self.i2c.writeto(self.PCA9685_ADDRESS, [MOTOR_OFF_L, off & 0xFF, (off >> 8) & 0xFF])

    def angle_to_pwm(self,angle):
        min_pulse_width_ms = 0.5  # Minimum pulse width in milliseconds (e.g., 0.5 ms)
        max_pulse_width_ms = 2.5  # Maximum pulse width in milliseconds (e.g., 2.5 ms)

        # Map angle (0-180) to pulse width (min_pulse_width_ms to max_pulse_width_ms)
        pulse_width_ms = min_pulse_width_ms + (angle / 180.0) * (max_pulse_width_ms - min_pulse_width_ms)

        # Convert pulse width to 12-bit value (0-4095)
        pulse_width_steps = int((pulse_width_ms / 20.0) * 4096)
        return pulse_width_steps
    
    # Control servo motor
    def control_servo(self, channel, tar_angle, curr_angle):
        step = self.speed if tar_angle > curr_angle else -self.speed
        for angle in range(curr_angle, tar_angle, step):
            pwm_value = self.angle_to_pwm(angle)
            self.set_pwm(channel, on=0, off=pwm_value)
            time.sleep(0.02)
            print(f"Setting servo on channel {channel} to {angle}° (PWM value: {pwm_value})")

    def control_servo_init(self, channel, angle):
        
        pwm_value = self.angle_to_pwm(angle)
        self.set_pwm(channel, on=0, off=pwm_value)

        print(f"Setting servo on channel {channel} to {angle}° (PWM value: {pwm_value})")

    