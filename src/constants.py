# Registers and i2c addresses for the LSM6DSL and LIS3MDL chips on the berryIMU.
# See datasheets for more info:
# LSM6DSL accelerometer and gyroscope: https://ozzmaker.com/wp-content/uploads/2020/08/lsm6dsl-datasheet.pdf
# LIS3MSL magnetometer: https://ozzmaker.com/wp-content/uploads/2020/08/lis3mdl.pdf

# constants for the LSM6DSL accelerometer and gyroscope
AG_ADDRESS = 0x6A  # i2c slave address

# register for gyroscope settings
CTRL2_G = 0x11

# registers for accelerometer settings
CTRL1_XL = 0x10
CTRL8_XL = 0x17

# output registers for accelerometer and gyroscope
OUTX_L_XL = 0x28
OUTX_L_G = 0x22

# hyroscope Gain value based on the set sensitivitiy level of 2000 dps
G_GAIN = 0.07

# accelerometer gain value Based on sensitvity of 8g
A_GAIN = 0.244

# Register addresses for the LIS3MDL magnetometer
CTRL_REG1 = 0x20
CTRL_REG2 = 0x21
CTRL_REG3 = 0x22
CTRL_REG4 = 0x23
OUT_X_L = 0x28
MAG_ADDRESS = 0x1C

# time between each sensor reading
DT = 0.08
