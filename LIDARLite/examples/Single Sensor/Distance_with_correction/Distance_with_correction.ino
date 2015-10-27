/* =============================================================================
  LIDAR-Lite v2: Corrections to get high precision using a LidarLite.

  This example demonstrates how to get high precision when taking measurements.
  The polynomial (first order) have to be calculed before using this script. 

  The library is in BETA, so subscribe to the github repo to recieve updates, or
  just check in periodically:
  https://github.com/PulsedLight3D/LIDARLite_v2_Arduino_Library

  To learn more read over lidarlite.cpp as each function is commented
=========================================================================== */

#include <Wire.h>
#include <LIDARLite.h>

LIDARLite myLidarLite;


// Correction polynomial is defined as first order factor, zero order factor.
// To define this function, I did a mean square regression on 60 values taken in my 
// use case, between 1.5m and 8.5m.
float calibration_factors[] = { 0.999, -19.14 };

float capture(int _howMuch){
  long total = 0;
  for(int i = 0; i<_howMuch; i++){
    total += myLidarLite.distance();
  }
  return float(float(total) / float(_howMuch));
}

// Compute function just uses factors of the function : y = 0.999x - 19.14.
// This is an EXAMPLE, with four lasers, I got four different functions. 
// Note : For most applications, the lineraity of the LIDAR-Lite is enough.
float compute(float _result, float *_factors){
  return _result*_factors[0] + _factors[1];
}

void setup() {
  Serial.begin(115200);
  myLidarLite.begin();
}

void loop() {
  float result = capture(100);
  float correct_result = compute(result, calibration_factors);
  Serial.print("result : ");
  Serial.print(result);
  Serial.print("\t");
  Serial.print("correct : ");
  Serial.println(correct_result);
}


