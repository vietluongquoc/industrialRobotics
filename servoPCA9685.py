from i2cpy import I2C

class ServoPCA9685:
    def __init__(self, address=0x40, bus=1, driver="ch341"):
        self.i2c = I2C(driver)
        self.address = address
        self.i2c.writeByte(self.address, 0x00, 0x00)  # Set mode1 to normal mode

    def set_pwm(self, channel, on, off):
        self.i2c.writeByte(self.address, 0x06 + 4 * channel, on & 0xFF)
        self.i2c.writeByte(self.address, 0x07 + 4 * channel, on >> 8)
        self.i2c.writeByte(self.address, 0x08 + 4 * channel, off & 0xFF)
        self.i2c.writeByte(self.address, 0x09 + 4 * channel, off >> 8)

    def set_servo_angle(self, channel, angle):
        pulse_length = int((angle / 180.0) * (4096 - 1))  # Convert angle to pulse length
        self.set_pwm(channel, 0, pulse_length)  # Set the pulse width for the servo

    def cleanup(self):
        self.i2c.close()  # Close the I2C connection when done