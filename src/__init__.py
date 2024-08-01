from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.components.movement_sensor import MovementSensor
from .berryimu3 import Berryimu3

Registry.register_resource_creator(MovementSensor.SUBTYPE, Berryimu3.MODEL, ResourceCreatorRegistration(Berryimu3.new, Berryimu3.validate))
