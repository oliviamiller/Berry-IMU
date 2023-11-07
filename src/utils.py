from . import constants
from viam.proto.common import Vector3


def init_accelerometer(i2cbus):
    # output data rate 3.33kHz, max reading 8g, BW = 400hz
    i2cbus.write_byte_data(constants.AG_ADDRESS, constants.CTRL1_XL, 0x9F)
    # low pass filter, Bw9, composite filtering
    i2cbus.write_byte_data(constants.AG_ADDRESS, constants.CTRL8_XL, 0xC8)


def init_gyroscope(i2cbus):
    # output data rate 3.33kHz, max reading 2000 deg/sec.
    i2cbus.write_byte_data(constants.AG_ADDRESS, constants.CTRL2_G, 0x9C)


def init_magnetometer(i2cbus):
    # Set to high performance mode, output rate of 10Hz
    i2cbus.write_byte_data(constants.MAG_ADDRESS, constants.CTRL_REG1, 0x50)
    # set scale to +/- 12 gaus
    i2cbus.write_byte_data(constants.MAG_ADDRESS, constants.CTRL_REG2, 0x40)
    # Enable continuous-conversion
    i2cbus.write_byte_data(constants.MAG_ADDRESS, constants.CTRL_REG3, 0x0)
    # set z-axis performance mode to high, little endian
    i2cbus.write_byte_data(constants.MAG_ADDRESS, constants.CTRL_REG4, 0x8)


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
