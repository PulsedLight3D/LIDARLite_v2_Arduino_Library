/* =============================================================================
  LIDAR-Lite v2: Single sensor, get distance as fast as possible

  This example file demonstrates how to take distance measurements as fast as
  possible, when you first plug-in a LIDAR-Lite into an Arduino it runs 250
  measurements per second (250Hz). Then if we setup the sensor by reducing the
  aquisiton record count by 1/3 and incresing the i2c communication speed from
  100kHz to 400kHz we get about 500 measurements per second (500Hz). Now if we
  throttle the reference and preamp stabilization processes during the distance
  measurement process we can increase the number of measurements to about 750
  per second (750Hz).

   The library is in BETA, so subscribe to the github repo to recieve updates, or
   just check in periodically:
   https://github.com/PulsedLight3D/LIDARLite_v2_Arduino_Library

   To learn more read over lidarlite.cpp as each function is commented
=========================================================================== */

#include <Wire.h>
#include <LIDARLite.h>

// Create a new LIDARLite instance
LIDARLite myLidarLite;

void setup() {
  Serial.begin(115200);

  //  First we want to set the aquisition count to 1/3 the default (works great for stronger singles)
  //  can be a little noisier (this is the "1"). Then we set the "true" to enable 400kHz i2c
  //  communication speed.

  myLidarLite.begin(1,true);
}

void loop() {

  //  Next we need to take 1 reading with preamp stabilization and reference pulse (these default to true)
  Serial.println(myLidarLite.distance());

  // Next lets take 99 reading without preamp stabilization and reference pulse (these read about 0.5-0.75ms faster than with)
  for(int i = 0; i < 99; i++){
    Serial.println(myLidarLite.distance(false,false));
  }
}
