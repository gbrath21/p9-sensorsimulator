#include <Wire.h>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_Sensor.h>

// Create sensor objects
Adafruit_ISM330DHCX ism330dhcx;
Adafruit_ADXL375 adxl375 = Adafruit_ADXL375(12345);

// Pin definitions for ESP32
#define ISM_CS 5  // Chip select for ISM330DHCX
#define ADXL_CS 4  // Chip select for ADXL375

// Data structure to hold sensor readings
struct SensorData {
  // ISM330DHCX data
  float accelX, accelY, accelZ;  // m/s²
  float gyroX, gyroY, gyroZ;     // rad/s
  float temp;                    // °C
  
  // ADXL375 data
  float highG_X, highG_Y, highG_Z; // m/s²
} sensorData;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("Windsurf Sensor Simulator - Initializing...");
  
  // Initialize I2C bus
  Wire.begin();
  
  // Initialize ISM330DHCX (6-DoF IMU)
  if (!ism330dhcx.begin_SPI(ISM_CS)) {
    Serial.println("ERROR: Failed to initialize ISM330DHCX IMU!");
    while (1) { delay(10); }
  }
  
  // Configure ISM330DHCX
  ism330dhcx.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  ism330dhcx.setGyroRange(ISMLSM6DS_GYRO_RANGE_2000_DPS);
  ism330dhcx.setAccelDataRate(LSM6DS_RATE_104_HZ);
  ism330dhcx.setGyroDataRate(LSM6DS_RATE_104_HZ);
  
  // Initialize ADXL375 (High-G Accelerometer)
  if (!adxl375.begin()) {
    Serial.println("ERROR: Failed to initialize ADXL375!");
    while (1) { delay(10); }
  }
  
  // Configure ADXL375
  adxl375.setRange(ADXL375_RANGE_200_G);
  
  Serial.println("Sensors initialized successfully!");
  Serial.println("------------------------------------");
  Serial.println("Time(ms) | ISM Accel(X,Y,Z) [m/s²] | Gyro(X,Y,Z) [rad/s] | ADXL Accel(X,Y,Z) [m/s²]");
  Serial.println("-------------------------------------------------------------------------------");
  
  delay(1000); // Give sensors time to stabilize
}

void readSensors() {
  sensors_event_t accel, gyro, temp;
  
  // Read ISM330DHCX data
  ism330dhcx.getEvent(&accel, &gyro, &temp);
  
  // Store ISM330DHCX data
  sensorData.accelX = accel.acceleration.x;
  sensorData.accelY = accel.acceleration.y;
  sensorData.accelZ = accel.acceleration.z;
  sensorData.gyroX = gyro.gyro.x;
  sensorData.gyroY = gyro.gyro.y;
  sensorData.gyroZ = gyro.gyro.z;
  sensorData.temp = temp.temperature;
  
  // Read ADXL375 data
  sensors_event_t event;
  adxl375.getEvent(&event);
  
  // Store ADXL375 data
  sensorData.highG_X = event.acceleration.x;
  sensorData.highG_Y = event.acceleration.y;
  sensorData.highG_Z = event.acceleration.z;
}

void printSensorData() {
  // Print timestamp
  Serial.print(millis());
  Serial.print(" | ");
  
  // Print ISM330DHCX accelerometer data
  Serial.print(sensorData.accelX, 2);
  Serial.print(", ");
  Serial.print(sensorData.accelY, 2);
  Serial.print(", ");
  Serial.print(sensorData.accelZ, 2);
  Serial.print(" | ");
  
  // Print ISM330DHCX gyroscope data
  Serial.print(sensorData.gyroX, 2);
  Serial.print(", ");
  Serial.print(sensorData.gyroY, 2);
  Serial.print(", ");
  Serial.print(sensorData.gyroZ, 2);
  Serial.print(" | ");
  
  // Print ADXL375 accelerometer data
  Serial.print(sensorData.highG_X, 2);
  Serial.print(", ");
  Serial.print(sensorData.highG_Y, 2);
  Serial.print(", ");
  Serial.println(sensorData.highG_Z, 2);
}

void loop() {
  // Read all sensors
  readSensors();
  
  // Print the sensor data
  printSensorData();
  
  // Update at approximately 10 Hz
  delay(100);
}
