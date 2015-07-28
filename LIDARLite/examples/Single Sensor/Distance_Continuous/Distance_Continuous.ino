/* =============================================================================
  LIDAR-Lite v2: Continous distance measurements

  This example file will demonstrate how to tell the sensor to take a continous
  set of readings by writing the speed and number of measruments directly to the
  sensor, freeing the micro-controller to read when it is ready. We will con-
  figure the MODE pin to pull low when a new measrument is available.

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
  myLidarLite.beginContinuous();
  pinMode(3, INPUT);
}

void loop() {
  if(!digitalRead(3)){
    Serial.println(myLidarLite.distanceContinuous());
  }
}
