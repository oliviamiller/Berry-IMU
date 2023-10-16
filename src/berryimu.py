from typing import ClassVar, Mapping, Tuple
from typing import ClassVar, Mapping, Any, Dict, Optional, List, cast
from typing_extensions import Self

from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName, Geometry
from viam.resource.base import ResourceBase
from viam.resource.types import Model, ModelFamily

from viam.components.movement_sensor import MovementSensor
from viam.logging import getLogger
from viam.errors import MethodNotImplementedError
from viam.proto.common import GeoPoint, Orientation, Vector3


LOGGER = getLogger(__name__)

class Berryimu(MovementSensor):
    MODEL: ClassVar[Model] = Model(ModelFamily("viam", "movement_sensor"), "berryimu")

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        berryimu = cls(config.name)
        berryimu.reconfigure(config, dependencies)
        return berryimu

    async def get_readings(
        self, *, extra: Optional[Mapping[str, Any]] = None, timeout: Optional[float] = None, **kwargs
    ) -> Mapping[str, Any]:
        return MethodNotImplementedError
    

    async def get_position(
        self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs
    ) -> Tuple[GeoPoint, float]:
         return MethodNotImplementedError

    async def get_linear_velocity(self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs) -> Vector3:
        return MethodNotImplementedError
    

    async def get_angular_velocity(self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs) -> Vector3:
         return MethodNotImplementedError
    
    async def get_linear_acceleration(
        self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs
    ) -> Vector3:
         return berryimu.readGYRz()

    async def get_compass_heading(self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs) -> float:
        return MethodNotImplementedError

    async def get_orientation(self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs) -> Orientation:
        return MethodNotImplementedError
    

    async def get_properties(self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs) -> MovementSensor.Properties:
        return MethodNotImplementedError
       
    async def get_accuracy(
        self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs
    ) -> Mapping[str, float]:
        return MethodNotImplementedError
        

        
    

        

        
