import asyncio
import sys
from viam.module.module import Module
from viam.components.movement_sensor import MovementSensor
from .berryimu import Berryimu

async def main():
    module = Module.from_args()
    module.add_model_from_registry(MovementSensor.SUBTYPE, Berryimu.MODEL)
    await module.start()

if __name__ == "__main__":
        asyncio.run(main())
        