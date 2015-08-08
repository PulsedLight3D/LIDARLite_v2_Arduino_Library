/* =============================================================================
  LIDAR-Lite v2: Single Sensor, get the correlation record as an array

  This example demostrates how to get the correlation record as an array and
  print it to the serial port

  The library is in BETA, so subscribe to the github repo to recieve updates, or
  just check in periodically:
  https://github.com/PulsedLight3D/LIDARLite_v2_Arduino_Library

  To learn more read over lidarlite.cpp as each function is commented
=========================================================================== */

#include <Wire.h>
#include <LIDARLite.h>

LIDARLite myLidarLite;
int correlationRecordArray[256];

void setup() {
  Serial.begin(115200);
  myLidarLite.begin();
}

void loop() {

  Serial.print("Distance: ");
  Serial.println(myLidarLite.distance());

  myLidarLite.correlationRecordToArray(correlationRecordArray);
  Serial.print("Correlation Record: ");

  for (int i=0;i<256;i++){
    Serial.print(correlationRecordArray[i]);
      Serial.print(", ");
  }
  Serial.println("Correlation Record Complete");
}
