#ifndef TOFS_HELPER_H
#define TOFS_HELPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <vl53l7cx_class.h>

// Define this if you would like verbose diagnostics on the TOF sensor startup
//#define TOFS_DEBUG

#define TOFS_DEFAULT_I2C_ADDRESS  0x29     // Time of flight sensor unconfigured default address
#define TOFS_LEFT_I2C_ADDRESS     0x2A     // Time of flight sensor (left) configured address
#define TOFS_RIGHT_I2C_ADDRESS    0x2B     // Time of flight sensor (right) configured address

enum tofs_state_t {
  TOFS_STATE_ABSENT = 0,
  TOFS_STATE_UNCONFIGURED = 1,
  TOFS_STATE_BEGAN = 2,
  TOFS_STATE_ASSIGNED = 3,
  TOFS_STATE_READY = 4
};

extern tofs_state_t tofs_left_state;
extern tofs_state_t tofs_right_state;

const char* STRING_FOR_TOFS_STATE(tofs_state_t state);

void tofs_setup();

bool check_left_sensor();
bool check_right_sensor();

#ifdef __cplusplus
}
#endif

#endif // TOFS_HELPER_H