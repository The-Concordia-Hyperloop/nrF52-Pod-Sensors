# Readme for the On Pod sensor system
The code sets up a BLE server with a custom service called "STM32 Hyperloop" and three characteristics: "IMU data", "pressure", and "battery". The IMU data characteristic sends 6 bytes of data from the LSM6DS3 sensor (2 bytes for each axis) and the pressure characteristic sends a single byte from the BMP280 sensor. The battery characteristic sends a single byte representing the battery voltage as a percentage of maximum voltage.

## Required Libraries
The following libraries are required:

ArduinoBLE.h: allows to create Bluetooth Low Energy peripherals.
Wire.h: library that allows to communicate with I2C / TWI devices.
SPI.h: library that allows to communicate with SPI devices.
LSM6DS3.h: library that allows to read data from the LSM6DS3 accelerometer and gyroscope.
Adafruit_BMP280.h: library that allows to read data from the BMP280 pressure sensor.

