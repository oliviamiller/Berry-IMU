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
from . import utils
import time
import math
import threading
import asyncio
from .orientation_vector import euler_angles_to_orientation_vector


LOGGER = getLogger(__name__)
dt = 0.08

class Berryimu(MovementSensor):
    MODEL: ClassVar[Model] = Model(ModelFamily("viam", "movement_sensor"), "berryimu")
    i2cbus: SMBus
    acceleration: Vector3
    accelerometer_raw: Vector3
    velocity: Vector3
    orientation: Orientation
    compass_heading: float
    accelerometer_angles: Vector3
    gyro_angles: Vector3
    euler_angles: Vector3
    calibrate: bool
    

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        berryimu = cls(config.name)
        berryimu.reconfigure(config, dependencies)
        b = threading.Thread(name='readings', target=berryimu.read)
        b.start()
        return berryimu
    
    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        bus = config.attributes.fields["i2c_bus"].number_value
        
        # open the i2c bus
        self.i2cbus = SMBus(bus)
        self.acceleration = Vector3(x = 0, y = 0, z = 0)
        self.accelerometer_raw = Vector3(x = 0, y = 0, z = 0)
        self.velocity = Vector3(x = 0, y = 0, z = 0)
        self.gyro_angles = Vector3(x = 0, y = 0, z = 0)
        self.compass_heading = 0
        self.accelerometer_angles =  Vector3(x = 0, y = 0, z = 0)
        self.euler_angles = {'roll': 0, 'pitch': 0, 'yaw': 0}
        self.orientation = Orientation(o_x = 0, o_y = 0, o_z = 0, theta = 0)
        self.calibrate = False

        if "hard_iron_x_max" in config.attributes.fields:
            self.calibrate = True
            self.hard_iron_x_max = config.attributes.fields["hard_iron_x_max"].number_value
        if "hard_iron_y_max" in config.attributes.fields:
            self.hard_iron_y_max = config.attributes.fields["hard_iron_y_max"].number_value
        if "hard_iron_x_min" in config.attributes.fields:
            self.hard_iron_x_min = config.attributes.fields["hard_iron_x_min"].number_value
        if "hard_iron_y_min" in config.attributes.fields:
            self.hard_iron_y_min = config.attributes.fields["hard_iron_y_min"].number_value
        if "soft_iron_x_max" in config.attributes.fields:
            self.soft_iron_x_max = config.attributes.fields["soft_iron_x_max"].number_value
        if "soft_iron_y_max" in config.attributes.fields:
            self.soft_iron_y_max = config.attributes.fields["soft_iron_y_max"].number_value
        if "soft_iron_x_min" in config.attributes.fields:
            self.soft_iron_x_min = config.attributes.fields["soft_iron_x_min"].number_value
        if "soft_iron_y_min" in config.attributes.fields:
            self.soft_iron_y_min = config.attributes.fields["soft_iron_y_min"].number_value

        utils.init_accelerometer(self.i2cbus)
        utils.init_gyroscope(self.i2cbus)
        utils.init_magnetometer(self.i2cbus)


        # initialize the euler angles to the accelerometer roll and pitch, which will be used in 
        # the complementary filtering
        self.get_acceleration()
        angles = self.get_acc_angles()
        self.euler_angles =  self.euler_angles = {'roll': angles.x, 'pitch': angles.y, 'yaw': 0}

        

    async def get_readings(
        self, *, extra: Optional[Mapping[str, Any]] = None, timeout: Optional[float] = None, **kwargs
    ) -> Mapping[str, Any]:
            readings = {}
            (av, la, comp, orient) = await asyncio.gather(
            self.get_angular_velocity(extra=extra, timeout=timeout),
            self.get_linear_acceleration(extra=extra, timeout=timeout),
            self.get_compass_heading(extra=extra, timeout=timeout),
            self.get_orientation(extra=extra, timeout=timeout),
            return_exceptions=True,
        )
            readings["angular_velocity"] = av
            readings["linear_acceleration"] = la
            readings["compass_heading"] = comp
            readings["orientation"] = orient
            readings["euler_angles"] = self.euler_angles
            return readings

    async def get_position(
        self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs
    ) -> Tuple[GeoPoint, float]:
         raise NotImplementedError

    async def get_linear_velocity(self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs) -> Vector3:
          raise NotImplementedError
    
    async def get_angular_velocity(self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs) -> Vector3:
          return self.velocity

    async def get_linear_acceleration(
        self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs
    ) -> Vector3:
          return self.acceleration

    async def get_compass_heading(self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs) -> float:
         return self.compass_heading

    async def get_orientation(self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs) -> Orientation:
        return self.orientation

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
        raise NotImplementedError
    

    
    def read(self):
        # This loop must run for the same amount of time each loop to accurately complete the complementary filter. 
        while True:
            now = time.time() 
            self.velocity = self.read_angular_velocity()
            self.compass_heading = self.calculate_compass_heading()
            self.acceleration = self.get_acceleration()
            self.orientation = self.calculate_orientation()
            elapsed = time.time() - now
            time.sleep(dt - elapsed) # full iteration takes 80 ms

    

    def calculate_compass_heading(self):
        raw_data = self.i2cbus.read_i2c_block_data(constants.MAG_ADDRESS,  constants.OUT_X_L, 6)
        values = utils.parse_output(raw_data)
        if self.calibrate:
            # hard iron distortion correction
            values.x -= (self.hard_iron_x_min + self.hard_iron_x_max) /2
            values.y -= (self.hard_iron_y_min + self.hard_iron_y_max) /2

            # #soft iron distortion correction 
            scale_y = self.soft_iron_y_max - self.soft_iron_y_min
            scale_x = self.soft_iron_x_max - self.soft_iron_x_min 
            ratio = scale_y / scale_x
            values.x *= ratio
            values.y *= ratio
        
        # tilt compensation.
        values.x = values.x * math.cos(self.euler_angles['pitch']) + values.z * math.sin(self.euler_angles['pitch'])

        values.y = values.x * math.sin(self.euler_angles['roll'])  * math.sin(self.euler_angles['pitch']) + values.y * math.cos(self.euler_angles['roll']) 
        - values.z * math.sin(self.euler_angles['roll']) * math.cos(self.euler_angles['pitch'])

        heading = 180 * math.atan2(values.y, values.x) / math.pi
        if heading < 0:
              heading += 360

        return heading
        


    def get_acceleration(self):
       raw_data = self.i2cbus.read_i2c_block_data(constants.AG_ADDRESS, constants.OUTX_L_XL, 6)
       self.accelerometer_raw = utils.parse_output(raw_data)
      
       #convert the raw readings to acceleration in m/s^2
       self.acceleration.x =  self.accelerometer_raw.x * constants.A_GAIN / 1000 * 9.8065
       self.acceleration.y =  self.accelerometer_raw.y * constants.A_GAIN / 1000 * 9.8065
       self.acceleration.z =  self.accelerometer_raw.z * constants.A_GAIN / 1000 * 9.8065

       return self.acceleration
    
    def read_angular_velocity(self):
       raw_data = self.i2cbus.read_i2c_block_data(constants.AG_ADDRESS, constants.OUTX_L_G, 6)
       values = utils.parse_output(raw_data)

       # convert raw readings to velocity in degrees/s
       self.velocity.x = values.x *  constants.G_GAIN
       self.velocity.y = values.y *  constants.G_GAIN
       self.velocity.z = values.z *  constants.G_GAIN

       return self.velocity


    def get_gyro_angles(self):
         # multiply the velocity by the time between readings to get the angle moved. 
        self.gyro_angles.x = math.radians(self.velocity.x * dt)
        self.gyro_angles.y = math.radians(self.velocity.y * dt)

        return self.gyro_angles
    
   # Get the accelerometer angles in radians 
    def get_acc_angles(self):

        # normalize the raw accelerometer values
        acc_x_norm = self.accelerometer_raw.x / math.sqrt(self.accelerometer_raw.x*self.accelerometer_raw.x + self.accelerometer_raw.y*self.accelerometer_raw.y + self.accelerometer_raw.z*self.accelerometer_raw.z)
        acc_y_norm = self.accelerometer_raw.y / math.sqrt(self.accelerometer_raw.x*self.accelerometer_raw.x + self.accelerometer_raw.y*self.accelerometer_raw.y + self.accelerometer_raw.z*self.accelerometer_raw.z)
        
        # calculate roll (x) and pitch (y)
        self.accelerometer_angles.x = math.asin(acc_x_norm)
        self.accelerometer_angles.y= math.asin(acc_y_norm/math.cos(self.accelerometer_angles.x))

        return self.accelerometer_angles
    
    def calculate_orientation(self):
        self.accelerometer_angles = self.get_acc_angles()
        self.gyro_angles = self.get_gyro_angles()


        ## complementary fiter is used to combine the readings from the gyro and accelerometer.
        self.euler_angles['roll'] = 0.98*(self.euler_angles['roll'] + self.gyro_angles.x) + 0.02 * self.accelerometer_angles.x
        self.euler_angles['pitch'] = 0.98*(self.euler_angles['pitch'] + self.gyro_angles.y) + 0.02 * self.accelerometer_angles.y
        self.euler_angles['yaw']= math.radians(self.compass_heading)

        # convert the euler angles to an orientation vector 
        orientation = euler_angles_to_orientation_vector(self.euler_angles['roll'] ,  self.euler_angles['pitch'],  self.euler_angles['yaw'])

        # convert theta radians to degrees
        orientation.theta = math.degrees(orientation.theta)
        return orientation


    async def close( self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs):
        self.i2cbus.close()
