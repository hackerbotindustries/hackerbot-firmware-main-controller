/*********************************************************************************
Hackerbot Industries, LLC
Created: February 2025

Configuration and helper functions for the time of flight sensors that are on
most Hackerbot models so that the robot can avoid obstacles taller than the 
built in bump and LiDAR sensors can "see"
*********************************************************************************/

#ifndef TOFS_HELPER_H
#define TOFS_HELPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <vl53l7cx_class.h>

#define TOFS_DEFAULT_I2C_ADDRESS  0x29     // Time of flight sensor unconfigured default address
#define TOF_LEFT_I2C_ADDRESS      0x2A     // Time of flight sensor (left) configured address
#define TOF_RIGHT_I2C_ADDRESS     0x2B     // Time of flight sensor (right) configured address

extern bool machine_mode;

enum tof_state_t {
  TOF_STATE_NOTCONNECTED = 0,
  TOF_STATE_DEFAULTI2CADDRESS = 1,
  TOF_STATE_INITIALIZED = 2,
  TOF_STATE_I2CADDRESSASSIGNED = 3,
  TOF_STATE_READY = 4
};

extern tof_state_t tof_left_state;
extern tof_state_t tof_right_state;

void tofs_setup();

bool check_left_sensor();
bool check_right_sensor();

#ifdef __cplusplus
}
#endif

#endif // TOFS_HELPER_H