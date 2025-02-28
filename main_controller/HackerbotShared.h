#ifndef HACKERBOTSHARED
#define HACKERBOTSHARED

#define AME_I2C_ADDRESS 0x5A              // Audio Mouth Eyes PCBA I2C address
#define DYN_I2C_ADDRESS 0x5B              // Dynamixel Controller I2C address
#define ARM_I2C_ADDRESS 0x5C              // Arm Controller I2C address

// I2C command addresses
// FIXME: need this to be sharable between projects - decide between a common library, a shared include directory (perhaps every sub-fw #include's a file from fw_main_controller?), or some other scheme
#define I2C_COMMAND_PING 0x01
#define I2C_COMMAND_VERSION 0x02
#define I2C_COMMAND_HEAD_IDLE 0x08
#define I2C_COMMAND_HEAD_LOOK 0x09
#define I2C_COMMAND_FACE_GAZE 0x0A
#define I2C_COMMAND_ARM_CALIBRATION 0x20
#define I2C_COMMAND_ARM_OPEN 0x21
#define I2C_COMMAND_ARM_CLOSE 0x22
#define I2C_COMMAND_ARM_ANGLE 0x25
#define I2C_COMMAND_ARM_ANGLES 0x26

// Where joint is 1...6
#define LIMIT_FOR_ARM_JOINT(joint) (joint == 6 ? 175.0 : 165.0)

uint16_t hb_ftoi(float val) {
  // multiply by 10 and add half of uint16_t's max value to prepare signed float to send as unsigned uint16_t
  return (uint16_t)((val * 10) + 0x7fff);
}

float hb_itof(uint16_t val) {
  // divide by 10 and subtract half of uint16_t's max value to convert unsigned uint16_t representation back to signed float
  return (float(val) - 0x7fff) * 0.1;
}

float hb_btof(byte* bytes) {
  return hb_itof((*bytes << 8) + *(bytes+1));
}

#endif // HACKERBOTSERIALCMD