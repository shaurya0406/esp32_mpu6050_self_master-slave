## Overview

1. Read external I2C MPU6050 Gyro/Accelerometer sensor.
2. Use one of ESP32’s I2C port (master mode) to read and write to another I2C port (slave mode) in ESP32.
A push button on the GPIO0 is attached to an interrupt and the Master sends the digital input to slave over I2C and the slave turns on the built_in LED on GPIO2                based on the received digital input.


### Hardware Required

To run this, you should have one ESP32 dev board (e.g. ESP32-WROVER Kit) or ESP32 core board (e.g. ESP32-DevKitC).

#### Pin Assignment:

**Note:** The following pin assignments are used by default, you can change these in the `menuconfig` .

|                  | SDA    | SCL    |
| ---------------- | ------ | ------ |
| ESP32 I2C Master | GPIO18 | GPIO19 |
| ESP32 I2C Slave  | GPIO4  | GPIO5  |
| MPU6050          | SDA    | SCL    |

- slave:
  - GPIO4 is assigned as the data signal of I2C slave port
  - GPIO5 is assigned as the clock signal of I2C slave port
- master:
  - GPIO18 is assigned as the data signal of I2C master port
  - GPIO19 is assigned as the clock signal of I2C master port

- Connection:
  - connect GPIO18 with GPIO4
  - connect GPIO19 with GPIO5
  - connect SDA/SCL of MPU6050 with GPIO18/GPIO19

**Note: ** There’s no need to add an external pull-up resistors for SDA/SCL pin, because the driver will enable the internal pull-up resistors.

### Configure the project

Open the project configuration menu (`make -j5 menuconfig`). Then go into `Example Configuration` menu.

- In the `I2C Master` submenu, you can set the pin number of SDA/SCL according to your board. Also you can modify the I2C port number and freauency of the master.
- In the `I2C Slave` submenu, you can set the pin number of SDA/SCL according to your board. Also you can modify the I2C port number and address of the slave.

### Build and Flash

Enter `make -j5 flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.
