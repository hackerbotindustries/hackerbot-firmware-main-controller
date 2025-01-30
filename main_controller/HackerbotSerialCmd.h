#ifndef HACKERBOTSERIALCMD
#define HACKERBOTSERIALCMD

#include <SerialCmd.h>

class HackerbotSerialCmd: public SerialCmd {
  public:
    HackerbotSerialCmd ( Stream &mySerial, char TermCh = SERIALCMD_CR, char * SepCh = ( char * ) SERIALCMD_COMMA ) : SerialCmd ( mySerial, TermCh, SepCh ) {};

    bool ReadNextFloat(float *value) {
      char* valStr = NULL;
      if ((valStr = ReadNext()) == NULL) { return false; }

      *value = atof(valStr);
      return true;
    }

    bool ReadNextUInt8(uint8_t *value) {
      char* valStr = NULL;
      if ((valStr = ReadNext()) == NULL) { return false; }

      *value = uint8_t(atoi(valStr));
      return true;
    }
};

#endif