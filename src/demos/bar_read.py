import time
import board
from adafruit_dps310.basic import DPS310

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
dps310 = DPS310(i2c, address=0x76)

# Initial pressure reading for sea level calibration (first power on gives 0m altitude assumption)
p0 = dps310.pressure

while True:
    temperature = dps310.temperature
    pressure = dps310.pressure

    # Calculate altitude using the barometric formula
    # Altitude in meters
    bar_alt = 44330.0 * (1.0 - pow(pressure / p0, 1.0 / 5.255))

    print("Temperature = %.2f *C" % temperature)
    print("Pressure = %.2f hPa" % pressure)
    print("Altitude = %.2f m" % bar_alt)

    time.sleep(1.0)
