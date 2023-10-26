from typing import ClassVar, Mapping, Tuple
from typing import ClassVar, Mapping, Any, Dict, Optional, List, cast
from typing_extensions import Self

from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.types import Model, ModelFamily

from viam.components.movement_sensor import MovementSensor
from viam.logging import getLogger
from viam.errors import MethodNotImplementedError
from viam.proto.common import GeoPoint, Orientation, Vector3
from smbus import SMBus
from . import constants
import time
import math
import asyncio
import threading




LOGGER = getLogger(__name__)

class Berryimu(MovementSensor):
    MODEL: ClassVar[Model] = Model(ModelFamily("viam", "movement_sensor"), "berryimu")
    i2cbus: SMBus
    acceleration: Vector3
    accelerometer_raw: Vector3
    mag_raw: Vector3
    velocity: Vector3
    orientation: Orientation
    compass_heading: float
    accelerometer_angles: Vector3
    calibrate: bool
    

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        berryimu = cls(config.name)
        bus = config.attributes.fields["i2c_bus"].number_value
        
      #   berryimu.serial_number = config.attributes.fields["serial_number"].string_value
        ## open the i2c bus
        berryimu.i2cbus = SMBus(bus)
        berryimu.acceleration = Vector3(x = 0, y = 0, z = 0)
        berryimu.accelerometer_raw = Vector3(x = 0, y = 0, z = 0)
        berryimu.mag_raw = Vector3(x = 0, y = 0, z = 0)
        berryimu.velocity = Vector3(x = 0, y = 0, z = 0)
        berryimu.orientation = Orientation(o_x=0, o_y=0, o_z=0, theta = 0)
        berryimu.compass_heading = 0
        berryimu.accelerometer_angles =  Vector3(x = 0, y = 0, z = 0)

        if "mag_x_max" in config.attributes.fields:
            berryimu.calibrate = True
            berryimu.mag_x_max = config.attributes.fields["mag_x_max"].number_value
        if "mag_y_max" in config.attributes.fields:
            berryimu.mag_y_max = config.attributes.fields["mag_y_max"].number_value
        if "mag_z_max" in config.attributes.fields:
            berryimu.mag_z_max = config.attributes.fields["mag_z_max"].number_value
        if "mag_x_min" in config.attributes.fields:
            berryimu.mag_x_min = config.attributes.fields["mag_x_min"].number_value
        if "mag_y_min" in config.attributes.fields:
            berryimu.mag_y_min = config.attributes.fields["mag_y_min"].number_value
        if "mag_z_min" in config.attributes.fields:
            berryimu.mag_z_min = config.attributes.fields["mag_z_min"].number_value



        berryimu._init_gyroscope()
        berryimu._init_accelerometer()
        berryimu._init_magnetometer()

        b = threading.Thread(name='readings', target=berryimu.read)
        b.start()

        return berryimu
    
    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        return None

    async def get_readings(
        self, *, extra: Optional[Mapping[str, Any]] = None, timeout: Optional[float] = None, **kwargs
    ) -> Mapping[str, Any]:
         raise NotImplementedError

    async def get_position(
        self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs
    ) -> Tuple[GeoPoint, float]:
         raise NotImplementedError

    async def get_linear_velocity(self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs) -> Vector3:
         # values = self.read_raw_gyro()
         return
    

    async def get_angular_velocity(self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs) -> Vector3:
          return self.velocity

    
    async def get_linear_acceleration(
        self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs
    ) -> Vector3:
          return self.acceleration

    async def get_compass_heading(self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs) -> float:
         return self.compass_heading

    async def get_orientation(self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs) -> Orientation:
        return self.calculate_orientation()
    

    async def get_properties(self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs) -> MovementSensor.Properties:
        return MovementSensor.Properties(linear_velocity_supported=False,
                                         angular_velocity_supported=True,
                                         orientation_supported=True,
                                         position_supported=False,
                                         compass_heading_supported=True,
                                         linear_acceleration_supported=True)
       
    async def get_accuracy(
        self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs
    ) -> Mapping[str, float]:
        return MethodNotImplementedError
    
    def _init_gyroscope(self):
        #output data rate 3.33kHz, max reading 2000 deg/sec.
        self.i2cbus.write_byte_data( constants.AG_ADDRESS,  constants.CTRL2_G, 0x9C)
    
    def _init_accelerometer(self):
        #output data rate 3.33kHz, max reading 8g, BW = 400hz 
        self.i2cbus.write_byte_data( constants.AG_ADDRESS,  constants.CTRL1_XL, 0x9F)
        # low pass filter, Bw9, composite filtering
        self.i2cbus.write_byte_data( constants.AG_ADDRESS,  constants.CTRL8_XL,0xC8)
        # Enable block data update, increment register address when multibyte read performed. 
        self.i2cbus.write_byte_data( constants.AG_ADDRESS,  constants.CTRL3_C, 0x44)

    
    def _init_magnetometer(self):
        #Set to high performance mode, output rate of 40Hz
        self.i2cbus.write_byte_data( constants.MAG_ADDRESS,  constants.CTRL_REG1, 0x58)
        # set scale to +/- 12 gaus 
        self.i2cbus.write_byte_data( constants.MAG_ADDRESS,  constants.CTRL_REG2,0x40)
        # Enable continuous-conversion
        self.i2cbus.write_byte_data( constants.MAG_ADDRESS,  constants.CTRL_REG3, 0x0)
        # set z-axis performance mode to high, little endian 
        self.i2cbus.write_byte_data(constants.MAG_ADDRESS,  constants.CTRL_REG4, 0x8)

    
    def read(self):
        while True:
            self.velocity = self.read_angular_velocity()
            self.compass_heading = self.calculate_compass_heading()
            self.acceleration = self.get_acceleration()
            time.sleep(0.1)

    

    def calculate_compass_heading(self):
        raw_data = self.i2cbus.read_i2c_block_data(constants.MAG_ADDRESS,  constants.OUT_X_L, 6)
        values = self.parse_output(raw_data)
        if self.calibrate:
            # hard iron distortion correction
            values.x -= (self.mag_x_min + self.mag_x_max) /2
            values.y -= (self.mag_y_min + self.mag_y_max) /2
            values.z -= (self.mag_z_min + self.mag_z_max) /2

            # remove soft iron distortion
            values.x = (values.x - self.mag_x_min) / (self.mag_x_max - self.mag_x_min) * 2 - 1
            values.y = (values.y - self.mag_y_min) / (self.mag_y_max - self.mag_y_min) * 2 - 1
            values.z = (values.x - self.mag_z_min) / (self.mag_z_max - self.mag_z_min) * 2 - 1

        heading = 180 * math.atan2(values.y, values.x) / math.pi
        if heading < 0:
              heading += 360
        return heading
        


    #Takes the raw data from mag, gyro, or acc
    #Returns the signed x,y,z vector it represents
    def parse_output(self, raw_data):
        values = {}
        values['x'] = raw_data[0] | raw_data[1] << 8
        values['y'] = raw_data[2] | raw_data[3] << 8
        values['z'] = raw_data[4] | raw_data[5] << 8
        for i in values:
            if values[i] & 0x8000:
                values[i] = -1 * ((~(values[i]) + 1) & 0xFFFF)
        vector = Vector3(x=values['x'], y=values['y'], z=values['z'])
        return vector


    def get_acceleration(self):
       raw_data = self.i2cbus.read_i2c_block_data(constants.AG_ADDRESS, constants.OUTX_L_XL, 6)
       self.accelerometer_raw = self.parse_output(raw_data)
      

       #then convert to m/s^2
       self.acceleration.x =  self.accelerometer_raw.x * constants.A_GAIN / 1000 * 9.8065
       self.acceleration.y =  self.accelerometer_raw.y * constants.A_GAIN / 1000 * 9.8065
       self.acceleration.z =  self.accelerometer_raw.z * constants.A_GAIN / 1000 * 9.8065

       return self.acceleration
    
    def read_angular_velocity(self):
       # read raw gyroscope values
       raw_data = self.i2cbus.read_i2c_block_data(constants.AG_ADDRESS, constants.OUTX_L_G, 6)
       values = self.parse_output(raw_data)

       # convert to m/s
       self.velocity.x = values.x *  constants.G_GAIN
       self.velocity.y = values.y *  constants.G_GAIN
       self.velocity.z = values.z *  constants.G_GAIN

       return self.velocity


    # multiply the velocity reading by the time of the loop - 0.02 seconds - to get angle.
    def get_gyro_angles(self):
        x = self.velocity.x * 0.02
        y = self.velocity.y * 0.02
        z = self.velocity.z * 0.02
        LOGGER.info("gyro angle x")
        LOGGER.info(x)
    
   # convert acceleramotor values to degrees
    def get_acc_angles(self):
       self.accelerometer_angles.x =  (math.atan2(self.accelerometer_raw.y, self.accelerometer_raw.z) + math.pi) * (180.0/ math.pi) - 180.0
       y =  (math.atan2(self.accelerometer_raw.z, self.accelerometer_raw.x) + math.pi) * (180.0/ math.pi)
       if y > 90:
        self.accelerometer_angles.y = y - 270
       else:
        self.accelerometer_angles.y = y + 90 

       


    def calculate_orientation(self):
        ## complemantary fiter
        self.get_acc_angles()
        self.orientation.o_x = 0.98*(self.orientation.o_x + self.velocity.x*0.02) + 0.02 * self.accelerometer_angles.x
        self.orientation.o_y = 0.98*(self.orientation.o_y + self.velocity.y*0.02) + 0.02 * self.accelerometer_angles.y
        return self.orientation


    
    


       





    
        

        
    

        

        
