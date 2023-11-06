# This script provides the values needed to calibrate the IMU's magnetometer
# Provides parametetrs to correct for hard and soft iron distortions 

import time
from smbus import SMBus
import constants
import utils 

hard_iron_x_min = 32767
hard_iron_y_min = 32767
hard_iron_x_max = -32767
hard_iron_y_max = -32767

soft_iron_x_min = 32767
soft_iron_y_min = 32767
soft_iron_x_max = -32767
soft_iron_y_max = -32767



def get_mag_raw_data(i2cbus):
        raw_data = i2cbus.read_i2c_block_data(constants.MAG_ADDRESS,  constants.OUT_X_L, 6)
        values = utils.parse_output(raw_data)
        return values

def get_mag_with_calibration(i2cbus):
        values = get_mag_raw_data(i2cbus)

        # hard iron correction, subtract the offset from the raw value.
        values.x -= (hard_iron_x_min + hard_iron_x_max) /2
        values.y -= (hard_iron_y_min + hard_iron_y_max) /2
        return values


i2cbus = SMBus(1)
utils.init_magnetometer(i2cbus)
print("calibrating the magnetometer. Start rotating the imu in all directions.\n")

# for 30 seconds, find the min and max for hard iron calibration.
t_end = time.time() + 30
while time.time() < t_end:
        values = get_mag_raw_data(i2cbus)

        if values.x > hard_iron_x_max:
         hard_iron_x_max = values.x
        if values.x < hard_iron_x_min:
         hard_iron_x_min = values.x
        if values.y > hard_iron_y_max:
         hard_iron_y_max = values.y
        if values.y < hard_iron_y_min:
         hard_iron_y_min = values.y
        time.sleep(0.025)

# with hard iron distortion applied find the new min and max
print("finding calibration values for soft iron distortion....keep rotating")
t_end = time.time() + 30
while time.time() < t_end: 
        values = get_mag_with_calibration(i2cbus)


        if values.x > soft_iron_x_max:
         soft_iron_x_max = values.x
        if values.x < soft_iron_x_min:
         soft_iron_x_min = values.x
        if values.y > soft_iron_y_max:
         soft_iron_y_max = values.y
        if values.y < soft_iron_y_min:
         soft_iron_y_min = values.y
        time.sleep(0.025)

print("copy and paste this config into the attributes for your berryIMU")
print(f"\"hard_iron_x_min\": {hard_iron_x_min},")
print(f"\"hard_iron_x_max\": {hard_iron_x_max},")
print(f"\"hard_iron_y_min\": {hard_iron_y_min},")
print(f"\"hard_iron_y_max\": {hard_iron_y_max},")
print(f"\"soft_iron_x_min\": {soft_iron_x_min},")
print(f"\"soft_iron_x_max\": {soft_iron_x_max},")
print(f"\"soft_iron_y_min\": {soft_iron_y_min},")
print(f"\"soft_iron_y_max\": {soft_iron_y_max}")
