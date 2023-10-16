from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.components.movement_sensor import MovementSensor
from .berryimu import Berryimu

Registry.register_resource_creator(MovementSensor.SUBTYPE, Berryimu.MODEL, ResourceCreatorRegistration(Berryimu.new))