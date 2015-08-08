/* =============================================================================
  LIDAR-Lite v2: Single Sensor, print the correlation record to the serial port

  This library demostrates how to print the correlation record to the serial 
  port

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
}

void loop() {

  Serial.print("Distance: ");
  Serial.println(myLidarLite.distance());

  myLidarLite.correlationRecordToSerial();
  Serial.println("Correlation Record Complete");
}
