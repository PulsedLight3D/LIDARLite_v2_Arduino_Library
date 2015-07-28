/* =============================================================================
  LIDAR-Lite v2: Change the I2C address of a single sensor.

  This example demonstrates how to chage the i2c address of a single sensor.

  The library is in BETA, so subscribe to the github repo to recieve updates, or
  just check in periodically:
  https://github.com/PulsedLight3D/LIDARLite_v2_Arduino_Library

  To learn more read over lidarlite.cpp as each function is commented
=========================================================================== */

#include <Wire.h>
#include <LIDARLite.h>

LIDARLite myLidarLite;

void setup() {
  Serial.begin(115200);
  myLidarLite.begin();

  // We assign the sensor the address of 0x66, and the false flag
  // tells the sensor to stop responding to 0x62

  myLidarLite.changeI2cAddress(0x66,false);
}

void loop() {

  // In order to talk to the sensor at the new address, we need
  // to set that value in the distance function. The first two "true"
  // bools tell the function to use its default values for dc stabilization
  // and reference

  Serial.println(myLidarLite.distance(true,true,0x66));
}
