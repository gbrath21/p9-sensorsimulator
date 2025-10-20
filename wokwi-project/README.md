# Windsurf Sensor Simulator

This project simulates an ESP32 Feather V2 board connected to two sensors:
1. Adafruit ISM330DHCX - 6 DoF IMU (Accelerometer and Gyroscope)
2. ADXL375 - High-G Accelerometer (±200g)

## Setup Instructions

1. Open this project in Wokwi (https://wokwi.com/)
2. The simulator will automatically load the configuration
3. Click "Start Simulation" to begin
4. Open the Serial Monitor to view sensor data

## Pin Connections

- **ISM330DHCX**
  - SCL → GPIO 22
  - SDA → GPIO 21
  - CS → GPIO 5

- **ADXL375**
  - SCL → GPIO 22 (shared I2C bus)
  - SDA → GPIO 21 (shared I2C bus)
  - CS → GPIO 4

## Expected Output

The serial monitor will display:
- Timestamp
- ISM330DHCX accelerometer data (X, Y, Z in m/s²)
- ISM330DHCX gyroscope data (X, Y, Z in rad/s)
- ADXL375 high-G accelerometer data (X, Y, Z in m/s²)

## Libraries Used

- Adafruit ISM330DHCX Library
- Adafruit ADXL375 Library
- Adafruit Unified Sensor
- Wire (I2C)

## Notes

- The simulator uses the ESP32-DevKit-V1 as it's the closest match in Wokwi
- I2C address conflicts are avoided by using different chip select (CS) pins
- The update rate is set to 10Hz to prevent serial monitor flooding
