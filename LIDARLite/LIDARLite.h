#include <Arduino.h>

class LIDARLite
{
  public:
      LIDARLite();
      void begin(int = 0, bool = false, bool = false, char = 0x62);
      void beginContinuous(bool = true, char = 0x04, char = 0xff, char = 0x62);
      void fast(char = 0x62);
      int distance(bool = true, bool = true, char = 0x62);
      int distanceContinuous(char = 0x62);
      void scale(char, char = 0x62);
      int velocity(char = 0x62);
      int signalStrength(char = 0x62);
      int* correlationRecord(int = 256, char = 0x62);
      unsigned char changeAddress(char, bool = false, char = 0x62);
      void write(char, char, char = 0x62);
      void read(char, int, byte*, bool, char);
  private:
      static bool errorReporting;
};
