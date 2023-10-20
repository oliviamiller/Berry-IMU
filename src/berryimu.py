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




LOGGER = getLogger(__name__)

class Berryimu(MovementSensor):
    MODEL: ClassVar[Model] = Model(ModelFamily("viam", "movement_sensor"), "berryimu")

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        berryimu = cls(config.name)
        ## open the i2c bus
        berryimu.i2cbus = SMBus(1)
        berryimu.magnetometer = Vector3(x = 0, y = 0, z = 0)
        # berryimu._init_gyroscope()
        # berryimu._init_accelerometer()
        berryimu._init_magnetometer()
        berryimu.read()
        berryimu.acceleration = 0
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
           raise NotImplementedError

    
    async def get_linear_acceleration(
        self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs
    ) -> Vector3:
          # self.read_raw_accelerometer()
          return

    async def get_compass_heading(self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs) -> float:
         raise NotImplementedError

    async def get_orientation(self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs) -> Orientation:
         raise NotImplementedError
    

    async def get_properties(self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs) -> MovementSensor.Properties:
        return MovementSensor.Properties(linear_velocity_supported=False,
                                         angular_velocity_supported=True,
                                         orientation_supported=False,
                                         position_supported=False,
                                         compass_heading_supported=False,
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

    

    def _read_raw_magnetometer(self):
        raw_data = self.i2cbus.read_i2c_block_data(constants.MAG_ADDRESS,  constants.OUT_X_L, 6)
        # LOGGER.info("mag raw data")
        # LOGGER.info(raw_data)
        x = int(raw_data[1]) << 8 | int(raw_data[0])
        signed_x = self.parse_Twos_Complement(x)
        
        # self.magnetometer.y = raw_data[2] | raw_data[3] << 8
        # self.magnetometer.z = raw_data[4] | raw_data[5] << 8
        LOGGER.info("x")
        LOGGER.info(x)
        LOGGER.info("signed x")
        LOGGER.info(signed_x)
    
     #Takes an unsigned 16-bit int representing a 2C number
    #Returns the signed number it represents
    def parse_Twos_Complement(self, unsigned_val):
        if unsigned_val & 0x8000:
            signed_val = (~unsigned_val + 1) & 0xFFFF
            return -1 * signed_val
        else:
            return unsigned_val
        
        
        
    
    def read(self):
         while 1:
              self._read_raw_magnetometer()
              self.calculate_compass_heading()
              time.sleep(1)


    def calculate_compass_heading(self):
         heading = 180 * math.atan2(self.magnetometer.y, self.magnetometer.x)
         if heading < 0:
              heading += 360
         #LOGGER.info(heading)
    





    # def read_raw_accelerometer(self):
    #    values = []
    #    raw_data = self.i2cbus.read_i2c_block_data(LSM6DSL.AG_ADDRESS, LSM6DSL.OUTX_L_XL, 6)
    #    values.append(raw_data[0] | raw_data[1] << 8)
    #    values.append(raw_data[2] | raw_data[3] << 8)
    #    values.append(raw_data[4] | raw_data[5] << 8)

    #    acc = self.get_acceleration(values)

    #    #LOGGER.info(values)
    #    return values
    
    # # convert to m/sec^2
    # def get_acceleration(self, readings: List):
    #     x = readings[0] * LSM6DSL.A_GAIN * 9.8065
    #     y = readings[1] * LSM6DSL.A_GAIN * 9.8065
    #     z = readings[2] * LSM6DSL.A_GAIN * 9.8065

    #     LOGGER.info(x)
    #     LOGGER.info(y)
    #     LOGGER.info(z)
    

    #     #LOGGER.info(values)
    #     self.acceleration = Vector3(x=x, y=y, z=z)
    
    # def read_raw_gyro(self):
    #    values = []
    #    raw_data = self.i2cbus.read_i2c_block_data(LSM6DSL.AG_ADDRESS, LSM6DSL.OUTX_L_G, 6)
    #    LOGGER.info(raw_data)
    #    values.append(raw_data[0] | (raw_data[1] << 8))
    #    values.append(raw_data[2] | raw_data[3] << 8)
    #    values.append(raw_data[4] | raw_data[5] << 8)

    #    LOGGER.info(values)
    #    vel = self.get_velocity(values)
    #    return values
    
    # def get_velocity(self, readings: List):
    #     x = readings[0] * LSM6DSL.G_GAIN
    #     y = readings[1] * LSM6DSL.G_GAIN
    #     z = readings[2] * LSM6DSL.G_GAIN

    #     LOGGER.info(x)
    #     LOGGER.info(y)
    #     LOGGER.info(z)
    
    #     return Vector3(x=x, y=y, z=z)
    


       





    
        

        
    

        

        
