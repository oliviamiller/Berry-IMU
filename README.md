# BerryIMU Module

This module implements the `rdk:movement_sensor` API in a `berryimu` model.
You can use this module to interface with the BerryIMU v3 [accelerometer, gyroscope](https://ozzmaker.com/wp-content/uploads/2020/08/lsm6dsl-datasheet.pdf), [magnetometer](https://ozzmaker.com/wp-content/uploads/2020/08/lis3mdl.pdf), and [barometric altitude sensor](https://www.mouser.com/pdfdocs/BST-BMP388-DS001-01.pdf) using an I2C connection on ARM64 systems.

## Requirements

- [BerryIMU v3](https://ozzmaker.com/product/berryimu-accelerometer-gyroscope-magnetometer-barometricaltitude-sensor/) movement sensor
- Single board computer running Linux Arm64
    - Enable I2C communication on your single board computer. If you are using a Raspberry Pi, see [Enable communication protocols](https://docs.viam.com/get-started/installation/prepare/rpi-setup/#enable-communication-protocols).
- Install the [Viam Python SDK](https://python.viam.dev/) on your single board computer:
```sh
pip install viam-sdk
```
- Calibrate your IMU sensor by running the following command in the BerryIMU module's `src` directory: 

```sh
python3 calibration.py
```

> [!TIP]
> During calibration, rotate the BerryIMU 360 degrees in all directions (X,Y, and Z) until the script returns calibration values.
> Note the returned calibration values, as you'll need to set these values into your movement sensor component's attributes during configuration. 

## Build and Run 

To use this module, follow the instructions to [add a module from the Viam Registry](https://docs.viam.com/registry/configure/#add-a-modular-resource-from-the-viam-registry) and select the `viam-labs:movement_sensor:berryimu` model from the [`berryimu` module]((https://app.viam.com/module/viam-labs/berryimu).

## Configure your BerryIMU movement sensor

> [!NOTE]
> Before configuring your movement sensor, you must [create a machine](https://docs.viam.com/fleet/machines/#add-a-new-machine).

Navigate to the **Config** tab of your machine's page in [the Viam app](https://app.viam.com/). Click on the **Components** subtab and click **Create component**.
Select the `movement-sensor` type, then select the `berryimu` model.
Enter a name for your movement sensor, click **Create**.

On the new component panel, copy and paste the following attribute template into your movement sensor's **Attributes** box:

```json
{
    "hard_iron_y_max": <INT>,
    "soft_iron_x_max": <INT>,
    "hard_iron_x_min": <INT>,
    "hard_iron_x_max": <INT>,
    "soft_iron_x_min": <INT>,
    "soft_iron_y_max": <INT>,
    "soft_iron_y_min": <INT>,
    "hard_iron_y_min": <INT>,
    "calibrate": <BOOL>,
    "i2c_bus": <STRING>
}
```

Then click **Save config** to save your changes.

> For more information, see [Configure a Machine](https://docs.viam.com/manage/configuration/).

 
### Attributes

Assign each attribute to the corresponding calibration value obtained when you calibrated the BerryIMU.

> [!NOTE]
> All `min` an `max` attributes are **optional** if the `calibrate` attribute is not set to `true` and **required** when calibration is enabled.
> The calibration values are obtained from running the `calibration.py` script.

|    **Name**                  |   **Type**    |  **Inclusion**| **Description** |
| ---------------------------- | ------------- | ------------- | --------------- |
| `hard_iron_y_max` |  int   | **Required**  | Maximum value for the y-axis hard iron distortion correction.|
| `soft_iron_x_max` |  int   | **Required**  | Maximum value for the x-axis soft iron distortion correction.|
| `hard_iron_x_min` |  int   | **Required** | Minimum value for the x-axis hard iron distortion correction.  |
| `hard_iron_x_max` |  int   | **Required** | Maximum value for the x-axis hard iron distortion correction.  |
| `soft_iron_x_min` |  int   | **Required** | Minimum value for the x-axis soft iron distortion correction.  |
| `soft_iron_y_max` |  int   | **Required** | Maximum value for the y-axis soft iron distortion correction.  |
| `soft_iron_y_min` |  int   | **Required** | Minimum value for the y-axis soft iron distortion correction.  |
| `hard_iron_y_min` |  int   | **Required** | Minimum value for the y-axis hard iron distortion correction.  |
| `calibrate` |  bool   | Optional | Flag indicating whether callibration is enabled to alibrate the magnetometer using the min and max values (true is enabled). |
| `i2c_bus` |  str   | **Required** | Name of the I2C bus wired to the IMU. |

#### Example configurations

```json
{
    "hard_iron_y_max": 1953,
    "soft_iron_x_max": 1069,
    "hard_iron_x_min": -1953,
    "hard_iron_x_max": 118,
    "soft_iron_x_min": -1069,
    "soft_iron_y_max": 1069,
    "soft_iron_y_min": -1056,
    "hard_iron_y_min": -1029,
    "calibrate": true,
    "i2c_bus": "1"
}
```
```json
{
    "calibrate": false,
    "i2c_bus": "1"
}
```
###  Wire your Berry IMU movement sensor

Wire the board pins outlined below to the corresponding pins on your IMU sensor. The IMU can be powered with the 3.3v pin, and also supports 5V.

| Board Pin  | Berry IMU Pin |
| ------------- | ------------- |
|      SCL      |      SCL      |
|      SDA      |      SDA      |
|      3.3V     |     3.3V      |

Verify the I2C connection by running the following command on your board:

```sh
    i2cdetect -y 1
```

If the IMU is wired correctly, the output of the command will be the I2C addresses `1c` and `6a`. This indicates that the IMU is successfully connected and detected by your system.

### Next Steps

- To test your movement sensor, go to the [**Control tab**](https://docs.viam.com/manage/fleet/robots/#control).
- To write code against your movement sensor, use one of the [available SDKs](https://docs.viam.com/program/).

## Troubleshooting

To troubleshoot if you encounter an `OSError: 5`, check your I2C connection. Ensure your wiring is correct and that the `i2cdetect` command successfully detects the device's I2C addresses.

