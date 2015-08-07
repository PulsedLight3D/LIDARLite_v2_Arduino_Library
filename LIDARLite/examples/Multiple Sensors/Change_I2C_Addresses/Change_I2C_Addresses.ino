/* =============================================================================
  LIDAR-Lite v2: Change the I2C address of multiple sensors with PWR_EN line

  This example demonstrates how to chage the i2c address of multiple sensors.

  The library is in BETA, so subscribe to the github repo to recieve updates, or
  just check in periodically:
  https://github.com/PulsedLight3D/LIDARLite_v2_Arduino_Library

  To learn more read over lidarlite.cpp as each function is commented
=========================================================================== */

#include <Wire.h>
#include <LIDARLite.h>

int sensorPins[] = {2,3,4}; // Array of pins connected to the sensor Power Enable lines
unsigned char addresses[] = {0x66,0x68,0x64};

LIDARLite myLidarLite;

void setup() {
  Serial.begin(115200);
  myLidarLite.begin();
  myLidarLite.changeAddressMultiPwrEn(3,sensorPins,addresses,false);
}

void loop() {
  Serial.print("Sensor 0x66: ");
  Serial.print(myLidarLite.distance(true,true,0x66));
  Serial.print(", Sensor 0x68: ");
  Serial.print(myLidarLite.distance(true,true,0x68));
  Serial.print(", Sensor 0x64: ");
  Serial.println(myLidarLite.distance(true,true,0x64));
}
