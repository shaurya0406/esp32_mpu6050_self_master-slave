ESP32-MPU6050
====================

This is a interface of esp32 with gyro/accelerometer mpu6050 over I2C. Also utilizing both I2C drivers of ESP32 to behave as master and slave.
A push button on the GPIO0 is attached to an interrupt and the Master sends the digital input to slave over I2C and the slave turns on the built_in LED on GPIO2 based on the received digital input.

Hardware Required

To run this example, you should have one ESP32 dev board (e.g. ESP32-WROVER Kit) or ESP32 core board (e.g. ESP32-DevKitC).

Pin Assignment:

Note: The following pin assignments are used by default, yout can change these in the menuconfig .

SDA	SCL
ESP32 I2C Master	GPIO18	GPIO19
ESP32 I2C Slave	GPIO4	GPIO5
MPU6050 SDA	SCL
slave:

GPIO4 is assigned as the data signal of I2C slave port
GPIO5 is assigned as the clock signal of I2C slave port
master:

GPIO18 is assigned as the data signal of I2C master port
GPIO19 is assigned as the clock signal of I2C master port
Connection:

connect GPIO18 with GPIO4
connect GPIO19 with GPIO5
connect SDA/SCL of MPU6050 with GPIO18/GPIO19
**Note: ** There’s no need to add an external pull-up resistors for SDA/SCL pin, because the driver will enable the internal pull-up resistors.
