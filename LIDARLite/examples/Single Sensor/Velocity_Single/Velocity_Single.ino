/* =============================================================================
  LIDAR-Lite v2: Single sensor, read velocity

  This example show how to read velocity with LIDAR-Lite "Blue Label" (V2)

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
  Serial.println(myLidarLite.velocity());
}
