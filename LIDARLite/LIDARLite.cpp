/* =============================================================================
  LIDARLite Arduino Library:

  The purpose of this library is two-fold:
  1.  Quick access all the basic functions of LIDAR-Lite via Arduino without
      worrying about specifics
  2.  By reading through this library, users of any platform will get an
      explanation of how to use the various functions of LIDAR-Lite and see an
      Arduino example along side.

  This libary was written by Austin Meyers (AK5A) with PulsedLight Inc. And was
  likely downloaded from:

  https://github.com/PulsedLight3D/LIDARLite_v2_Arduino_Library

  Visit http://pulsedlight3d.com for documentation and support requests

  To Do
  ------------------------------------------------------------------------------
  - Test correlation record by graphing it
  - New Functions
    - Multisensor address change by serial number
    - Multisensor address change by broadcast to all to get serial, then write
      to serial number
    - Sensor get serial number! (do this then Change address)

  Changelog
  ------------------------------------------------------------------------------
  - 7/23/15:
    - Removed "fast" command, redudant
  - 7/17/15: Initial Commit

============================================================================= */

#include <Arduino.h>
#include <Wire.h>
#include <stdarg.h>
#include "LIDARLite.h"

/* =============================================================================

  This var set in setup and used in the read function. It reads the value of
  register 0x40, used largely for sending debugging requests to PulsedLight

============================================================================= */
bool LIDARLite::errorReporting = false;

LIDARLite::LIDARLite(){}

/* =============================================================================

  Begin

  Starts the sensor and I2C

  Process
  ------------------------------------------------------------------------------
  1.  Turn on error reporting, off by default
  2.  Start Wire (i.e. turn on I2C)
  3.  Enable 400kHz I2C, 100kHz by default
  4.  Set configuration for sensor

  Parameters
  ------------------------------------------------------------------------------
  - configuration: set the configuration for the sensor
    - default or 0 = equivelent to writing 0x00 to 0x00, i.e. full reset of
      sensor, if you write nothing for configuration or 0, the sensor will init-
      iate normally
    - 1 = high speed setting, set the aquisition count to 1/3 the default (works
      great for stronger singles) can be a little noisier
  - fasti2c: if true i2c frequency is 400kHz, default is 100kHz
  - showErrorReporting: if true reads with errors will print the value of 0x40,
    used primarily for debugging purposes by PulsedLight
  - LidarLiteI2cAddress (optional): Default: 0x62, the default LIDAR-Lite
    address. If you change the address, fill it in here.

============================================================================= */
void LIDARLite::begin(int configuration, bool fasti2c, bool showErrorReporting, char LidarLiteI2cAddress){
  errorReporting = showErrorReporting;
  Wire.begin();
  if(fasti2c){
    #if ARDUINO >= 157
      Wire.setClock(400000UL); // Set I2C frequency to 400kHz, for the Due
    #else
      TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
    #endif
  }
  switch (configuration){
    case 0:
    break;
    case 1:
      write(0x04,0x00,LidarLiteI2cAddress);
    break;
  }
}

/* =============================================================================

  Distance

  Process
  ------------------------------------------------------------------------------
  1.  Write 0x04 to register 0x00 to initiate an aquisition.
  2.  Read register 0x01 (this is handled in the read() command)
      - if the first bit is "1" then the sensor is busy, loop until the first
        bit is "0"
      - if the first bit is "0" then the sensor is ready
  3.  Read two bytes from register 0x8f and save
  4.  Shift the FirstValueFrom0x8f << 8 and add to SecondValueFrom0x8f This new
      value is the distance.

  Parameters
  ------------------------------------------------------------------------------
  - stablizePreampFlag (optional): Default: true, take aquisition with DC
    stabilization/correction. If set to false, it will read
  - faster, but you will need to sabilize DC every once in awhile (ex. 1 out of
    every 100 readings is typically good).
  - LidarLiteI2cAddress (optional): Default: 0x62, the default LIDAR-Lite
    address. If you change the address, fill it in here.

  Example Arduino Usage
  ------------------------------------------------------------------------------
  1.  // take a reading with DC stabilization and the 0x62 default i2c address
      myLidarLiteInstance.distance();

  2.  // take a reading without DC stabilization and the 0x62 default i2c address
      myLidarLiteInstance.distance(false);

  3.  // take a reading with DC stabilization and a custom i2c address of 0x66
      myLidarLiteInstance.distance(true,0x66);

  Notes
  ------------------------------------------------------------------------------
    Autoincrement: A note about 0x8f vs 0x0f
    | Set the highest bit of any register to "1" if you set the high byte of a
    | register and then take succesive readings from that register, then LIDAR-
    | Lite automatically increments the register one for each read. An example:
    | If we want to read the high and low bytes for the distance, we could take
    | two single readings from 0x0f and 0x10, or we could take 2 byte read from
    | register 0x8f. 0x8f = 10001111 and 0x0f = 00001111, meaning that 0x8f is
    | 0x0f with the high byte set to "1", ergo it autoincrements.

============================================================================= */
int LIDARLite::distance(bool stablizePreampFlag, bool takeReference, char LidarLiteI2cAddress){
  if(stablizePreampFlag){
    write(0x00,0x04,LidarLiteI2cAddress); // Take acquisition & correlation processing with DC correction
  }else{
    write(0x00,0x03,LidarLiteI2cAddress); // Take acquisition & correlation processing without DC correction
  }
  byte distanceArray[2]; // Array to store high and low bytes of distance
  read(0x8f,2,distanceArray,true,LidarLiteI2cAddress); // Read two bytes from register 0x8f. (See autoincrement note above)
  int distance = (distanceArray[0] << 8) + distanceArray[1]; // Shift high byte and add to low byte
  return(distance);
}

/* =============================================================================
  Velocity Scaling

  Measurement | Velocity        | Register        | velocityScalingValue
  Period (ms) | Scaling (m/sec) | 0x68 Load Value |
  :-----------| :---------------| :---------------| :-------------------
  100         | 0.10 m/s        | 0xC8 (default)  | 1
  40          | 0.25 m/s        | 0x50            | 2
  20          | 0.50 m/s        | 0x28            | 3
  10          | 1.00 m/s        | 0x14            | 4

  Process
  ------------------------------------------------------------------------------
  1. Write the velocity scaling value from the table above to register 0x68

  Parameters
  ------------------------------------------------------------------------------
  - velocityScalingValue: interger to choose the velocity scaling value, refer
    to the table above
  - LidarLiteI2cAddress (optional): Default: 0x62, the default LIDAR-Lite
    address. If you change the address, fill it in here.

  Example Usage
  ------------------------------------------------------------------------------
  1.  // By default you don't need to set the scaling value, the sensor defaults
      // to 0xC8 for register 0x68 or 0.10m/s
  2.  // Set the velocity scaling to 1m/s
      myLidarLiteInstance.scale(4);

  =========================================================================== */

void LIDARLite::scale(char velocityScalingValue, char LidarLiteI2cAddress){
  unsigned char scale[] = {0xC8, 0x50, 0x28, 0x14};
  write(0x68,scale[velocityScalingValue],LidarLiteI2cAddress);
}

/* =============================================================================
  Velocity

  A velocity is measured by observing the change in distance over a fixed time
  period. The default time period is 100 ms resulting in a velocity calibration
  of .1 m/s. Velocity mode is selected by setting the most significant bit of
  internal register 4 to one. When a distance measurement is initiated by writ-
  ing a 3 or 4 (no dc compensation/or update compensation respectively) to com-
  mand register 0, two successive distance measurements result with a time delay
  defined by the value loaded into register at address 0x68.

  Process
  ------------------------------------------------------------------------------
  1.  Write 0x04 to register 0x00 to initiate an aquisition.
  2.  Write 0x80 to register 0x04 to switch to velocity mode
  3.  Read register 0x01
      - if the first bit is "1" then the sensor is busy, loop until the first
        bit is "0"
      - if the first bit is "0" then the sensor is ready
  4.  Read one bytes from register 0x09 and save

  Parameters
  ------------------------------------------------------------------------------
  - LidarLiteI2cAddress (optional): Default: 0x62, the default LIDAR-Lite
    address. If you change the address, fill it in here.

  Example Usage
  ------------------------------------------------------------------------------
  1.  // Basic usage with default i2c address
      myLidarLiteInstance.velocity();
  2.  // Get velocity with custom i2c address of 0x66
      myLidarLiteInstance.velocity(0x66);

  =========================================================================== */
int LIDARLite::velocity(char LidarLiteI2cAddress){
  write(0x00,0x04,LidarLiteI2cAddress); // Write 0x04 to register 0x00 to start getting distance readings
  write(0x04,0x80,LidarLiteI2cAddress); // Write 0x80 to 0x04 to switch on velocity mode
  byte myArray[1]; // Array to store bytes from read function
  read(0x09,1,myArray,true,LidarLiteI2cAddress); // Read 1 byte from register 0x09 to get velocity measurement
  return((int)((char)myArray[0])); // Convert 1 byte to char and then to int to get signed int value for velocity measurement
}

/* =============================================================================
  Correlation Record
  =========================================================================== */
int* LIDARLite::correlationRecord(int numberOfReadings, char LidarLiteI2cAddress){
  int *correlationRecord;
  byte my_read_val[2];
  int my_Value = 0;
  write(0x5d,0xc0,LidarLiteI2cAddress); // selects memory bank
  write(0x40, 0x07,LidarLiteI2cAddress); // sets test mode select
  for(int i = 0; i<numberOfReadings; i++){
    read(0xd2,2,my_read_val,false,LidarLiteI2cAddress); // added to select single byte
    my_Value = (int)my_read_val[0];
    if((int)my_read_val[1] == 1){ // if upper byte lsb is set, the value is negative
      my_Value |= 0xff00;
    }
    Serial.println(my_Value);
    correlationRecord[i] = my_Value;
  }
  write(0x40,0x00,LidarLiteI2cAddress); // Null command to control register
  return correlationRecord;

  // numberOfReadings = 0;
  // int elements[] = {256,384,512,640,768,896,1024};
  // byte my_read_val[2];
  // int my_Value = 0;
  //
  // //llWireWrite(0x51,0x10); // points to the base of the correlation record address
  // llWireWrite(0x5d,0xc0); // selects memory bank
  // llWireWrite(0x40, 0x07); // sets test mode select
  //
  // for(int i = 0; i<300; i++){
  //   llWireReadCR(0xd2,2,my_read_val); // added to select single byte
  //   my_Value = (int)my_read_val[0];
  //   if(my_read_val[1] ==1){ // if upper byte lsb is set, the value is negative
  //     my_Value |= 0xff00;
  //   }
  //   Serial.print(my_Value);
  //   Serial.println("*");
  // }
  // llWireWrite(0x40,0x00); // Null command to control register
  // Serial.println(999999);
}

/* =============================================================================
  =========================================================================== */
unsigned char LIDARLite::changeAddress(char newI2cAddress,  bool disablePrimaryAddress, char LidarLiteDefaultAddress){
  unsigned char serialNumber[2];
  read(0x96,2,serialNumber,false,LidarLiteDefaultAddress);
  write(0x18,serialNumber[0],LidarLiteDefaultAddress);
  write(0x19,serialNumber[1],LidarLiteDefaultAddress);
  write(0x1a,newI2cAddress,LidarLiteDefaultAddress);
  if(disablePrimaryAddress){
    write(0x1e,0x08,LidarLiteDefaultAddress);
  }else{
    write(0x1e,0x00,LidarLiteDefaultAddress);
  }
  return newI2cAddress;
}

/* =============================================================================
  =========================================================================== */
void LIDARLite::write(char myAddress, char myValue, char LidarLiteI2cAddress){
  Wire.beginTransmission((int)LidarLiteI2cAddress);
  Wire.write((int)myAddress);
  Wire.write((int)myValue);
  int nackCatcher = Wire.endTransmission();
  if(nackCatcher != 0){Serial.println("> nack");}
  delay(1);
}

/* =============================================================================
  =========================================================================== */
void LIDARLite::read(char myAddress, int numOfBytes, byte arrayToSave[2], bool monitorBusyFlag, char LidarLiteI2cAddress){
  int busyFlag = 0;
  if(monitorBusyFlag){
    int busyFlag = 1;
  }
  int busyCounter = 0;
  while(busyFlag != 0){
    Wire.beginTransmission((int)LidarLiteI2cAddress);
    Wire.write(0x01);
    int nackCatcher = Wire.endTransmission();
    if(nackCatcher != 0){Serial.println("> nack");}
    Wire.requestFrom((int)LidarLiteI2cAddress,1);
    busyFlag = bitRead(Wire.read(),0);
    busyCounter++;
    if(busyCounter > 9999){
      if(errorReporting){
        int errorExists = 0;
        Wire.beginTransmission((int)LidarLiteI2cAddress);
        Wire.write(0x01);
        int nackCatcher = Wire.endTransmission();
        if(nackCatcher != 0){Serial.println("> nack");}
        Wire.requestFrom((int)LidarLiteI2cAddress,1);
        errorExists = bitRead(Wire.read(),0);
        if(errorExists){
          unsigned char errorCode[] = {0x00};
          Wire.beginTransmission((int)LidarLiteI2cAddress);    // Get the slave's attention, tell it we're sending a command byte
          Wire.write(0x40);
          delay(20);
          int nackCatcher = Wire.endTransmission();                  // "Hang up the line" so others can use it (can have multiple slaves & masters connected)
          if(nackCatcher != 0){Serial.println("> nack");}
          Wire.requestFrom((int)LidarLiteI2cAddress,1);
          errorCode[0] = Wire.read();
          delay(10);
          Serial.print("> Error Code from Register 0x40: ");
          Serial.println(errorCode[0]);
          delay(20);
        }
       }
      goto bailout;
    }
  }
  if(busyFlag == 0){
    Wire.beginTransmission((int)LidarLiteI2cAddress);
    Wire.write((int)myAddress);
    int nackCatcher = Wire.endTransmission();
    if(nackCatcher != 0){Serial.println("NACK");}
    Wire.requestFrom((int)LidarLiteI2cAddress, numOfBytes);
    int i = 0;
    if(numOfBytes <= Wire.available()){
      while(i < numOfBytes){
        arrayToSave[i] = Wire.read();
        i++;
      }
    }
  }
  if(busyCounter > 9999){
    bailout:
      busyCounter = 0;
      Serial.println("> Bailout");
  }
}
