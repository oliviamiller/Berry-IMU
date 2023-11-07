from viam.proto.common import Vector3

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

def init_accelerometer(i2cbus):
    # output data rate 3.33kHz, max reading 8g, BW = 400hz
    i2cbus.write_byte_data(AG_ADDRESS, CTRL1_XL, 0x9F)
    # low pass filter, Bw9, composite filtering
    i2cbus.write_byte_data(AG_ADDRESS, CTRL8_XL, 0xC8)


def init_gyroscope(i2cbus):
    # output data rate 3.33kHz, max reading 2000 deg/sec.
    i2cbus.write_byte_data(AG_ADDRESS, CTRL2_G, 0x9C)


def init_magnetometer(i2cbus):
    # Set to high performance mode, output rate of 10Hz
    i2cbus.write_byte_data(MAG_ADDRESS, CTRL_REG1, 0x50)
    # set scale to +/- 12 gaus
    i2cbus.write_byte_data(MAG_ADDRESS, CTRL_REG2, 0x40)
    # Enable continuous-conversion
    i2cbus.write_byte_data(MAG_ADDRESS, CTRL_REG3, 0x0)
    # set z-axis performance mode to high, little endian
    i2cbus.write_byte_data(MAG_ADDRESS, CTRL_REG4, 0x8)

# returns raw magnetometer values.
def get_mag_raw_data(i2cbus):
    raw_data = i2cbus.read_i2c_block_data(MAG_ADDRESS, OUT_X_L, 6)
    values = parse_output(raw_data)
    return values


# Takes the raw data from mag, gyro, or acc
# Returns the signed x,y,z vector it represents
def parse_output(raw_data):
    values = {}
    values["x"] = raw_data[0] | raw_data[1] << 8
    values["y"] = raw_data[2] | raw_data[3] << 8
    values["z"] = raw_data[4] | raw_data[5] << 8
    for i in values:
        if values[i] & 0x8000:
            values[i] = -1 * ((~(values[i]) + 1) & 0xFFFF)
    vector = Vector3(x=values["x"], y=values["y"], z=values["z"])
    return vector
