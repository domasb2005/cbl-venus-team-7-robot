import board
import busio
import adafruit_vl53l0x

# Create I2C interface using Blinka
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize the sensor
vl53 = adafruit_vl53l0x.VL53L0X(i2c)

# Read distance in millimeters
while True:
    print("Distance: {}mm".format(vl53.range))
    time.sleep(1)
