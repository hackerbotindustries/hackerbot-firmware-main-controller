/*********************************************************************************
Hackerbot Industries, LLC
Created By: Ian Bernstein
Created:    April 2024
Updated:    April 1, 2025

This file adds some supplimental helper functions related to the SerialCmd libary

Special thanks to the following for their code contributions to this codebase:
Ian Bernstein - https://github.com/arobodude
Randy Beiter - https://github.com/rbeiter
*********************************************************************************/


#ifndef SERIALCMDHELPER_H
#define SERIALCMDHELPER_H

#include <SerialCmd.h>

class SerialCmdHelper: public SerialCmd {
  public:
    SerialCmdHelper (Stream &mySerial, char TermCh = SERIALCMD_CR, char * SepCh = (char *) SERIALCMD_COMMA) : SerialCmd (mySerial, TermCh, SepCh) {};

    bool ReadNextFloat(float *value) {
      char* valStr = NULL;
      if ((valStr = ReadNext()) == NULL) {
        return false;
      }

      *value = atof(valStr);
      return true;
    }

    bool ReadNextUInt8(uint8_t *value) {
      char* valStr = NULL;
      if ((valStr = ReadNext()) == NULL) {
        return false;
      }

      *value = uint8_t(atoi(valStr));
      return true;
    }
};

#endif