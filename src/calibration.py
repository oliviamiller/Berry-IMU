import time
from smbus import SMBus
import constants

x_min = 32767
y_min = 32767
z_min = 32767
x_max = -32767
y_max = -32767
z_max = -32767

def get_mag_raw_data(i2cbus):
        raw_data = i2cbus.read_i2c_block_data(constants.MAG_ADDRESS,  constants.OUT_X_L, 6)
        values = parse_output(raw_data)
        return values

#Takes the raw data from mag, gyro, or acc
#Returns the signed x,y,z vector it represents
def parse_output(raw_data):
        values = {}
        values['x'] = raw_data[0] | raw_data[1] << 8
        values['y'] = raw_data[2] | raw_data[3] << 8
        values['z'] = raw_data[4] | raw_data[5] << 8
        for i in values:
            if values[i] & 0x8000:
                values[i] = -1 * ((~(values[i]) + 1) & 0xFFFF)
        return values

def init_magnetometer(i2cbus):
        #Set to high performance mode, output rate of 40Hz
        i2cbus.write_byte_data( constants.MAG_ADDRESS,  constants.CTRL_REG1, 0x58)
        # set scale to +/- 12 gaus 
        i2cbus.write_byte_data( constants.MAG_ADDRESS,  constants.CTRL_REG2,0x40)
        # Enable continuous-conversion
        i2cbus.write_byte_data( constants.MAG_ADDRESS,  constants.CTRL_REG3, 0x0)
        # set z-axis performance mode to high, little endian 
        i2cbus.write_byte_data(constants.MAG_ADDRESS,  constants.CTRL_REG4, 0x8)

i2cbus = SMBus(1)
init_magnetometer(i2cbus)
print("calibrating the magnetometer. Move the imu in a circle\n")
print("press crtl c when done")
try:
    while True:
        values = get_mag_raw_data(i2cbus)

        if values['x'] > x_max:
         x_max = values['x']
        if values['x'] < x_min:
         x_min = values['x']
        if values['y'] > y_max:
         y_max = values['y']
        if values['y'] < y_min:
         y_min = values['y']
        if values['z'] > z_max:
         z_max = values['z']
        if values['z'] < z_min:
         z_min = values['z']
        #Sleep for 2.5ms
        time.sleep(0.025)
except KeyboardInterrupt:
   print("copy and paste this config into the attributes for your berryIMU")
   print(f"\"mag_x_min\": {x_min},")
   print(f"\"mag_x_max\": {x_max},")
   print(f"\"mag_y_min\": {y_min},")
   print(f"\"mag_y_max\": {y_max},")
   print(f"\"mag_z_min\": {z_min},")
   print(f"\"mag_z_max\": {z_max}")


		




		
	

