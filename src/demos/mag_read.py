import smbus2
import time

bus = smbus2.SMBus(1)

#I2C address
QMC5883L_ADDRESS = 0x0D

# QMC5883L registers
QMC5883L_REG_OUT_X_LSB = 0x00
QMC5883L_REG_OUT_X_MSB = 0x01
QMC5883L_REG_OUT_Y_LSB = 0x02
QMC5883L_REG_OUT_Y_MSB = 0x03
QMC5883L_REG_OUT_Z_LSB = 0x04
QMC5883L_REG_OUT_Z_MSB = 0x05
QMC5883L_REG_STATUS = 0x06
QMC5883L_REG_CONFIG_1 = 0x09
QMC5883L_REG_CONFIG_2 = 0x0A
QMC5883L_REG_RESET = 0x0B

# QMC5883L configuration settings
QMC5883L_MODE_CONTINUOUS = 0x01
QMC5883L_ODR_200HZ = 0x0C
QMC5883L_RNG_8G = 0x10
QMC5883L_OSR_512 = 0x00

def setup_qmc5883l():
    # Reset
    bus.write_byte_data(QMC5883L_ADDRESS, QMC5883L_REG_RESET, 0x01)
    time.sleep(0.1)

    # Configure
    bus.write_byte_data(QMC5883L_ADDRESS, QMC5883L_REG_CONFIG_1, QMC5883L_MODE_CONTINUOUS | QMC5883L_ODR_200HZ | QMC5883L_RNG_8G | QMC5883L_OSR_512)
    bus.write_byte_data(QMC5883L_ADDRESS, QMC5883L_REG_CONFIG_2, 0x00)  # Disable interrupt pin

def read_qmc5883l():
    data = bus.read_i2c_block_data(QMC5883L_ADDRESS, QMC5883L_REG_OUT_X_LSB, 6)

    # Convert the data
    x = (data[1] << 8) | data[0]
    if x > 32767:
        x -= 65536
    y = (data[3] << 8) | data[2]
    if y > 32767:
        y -= 65536
    z = (data[5] << 8) | data[4]
    if z > 32767:
        z -= 65536

    return x, y, z

if __name__ == "__main__":
    setup_qmc5883l()
    while True:
        x, y, z = read_qmc5883l()
        print(f"X: {x}, Y: {y}, Z: {z}")
        time.sleep(0.01)

