/*********************************************************************************
Hackerbot Industries, LLC
Created: February 2025

Configuration and helper functions for the time of flight sensors that are on
most Hackerbot models so that the robot can avoid obstacles taller than the 
built in bump and LiDAR sensors can "see"
*********************************************************************************/

#include <Wire.h>
#include "SerialCmd_Helper.h"
#include "ToFs_Helper.h"

extern SerialCmdHelper mySerCmd; // defined in the main sketch

VL53L7CX sensor_vl53l7cx_right(&Wire, 3); // LPn Enable Pin on D3
VL53L7CX sensor_vl53l7cx_left(&Wire, 10); // Requires LPn to be set so using unused pin D10

tof_state_t tof_left_state = TOF_STATE_NOTCONNECTED;
tof_state_t tof_right_state = TOF_STATE_NOTCONNECTED;

// Tof configuration parameters
void set_configuration(VL53L7CX *sensor) {
    sensor->vl53l7cx_set_resolution(VL53L7CX_RESOLUTION_4X4); // VL53L7CX_RESOLUTION_4X4, VL53L7CX_RESOLUTION_8X8
    sensor->vl53l7cx_set_target_order(VL53L7CX_TARGET_ORDER_CLOSEST);
    sensor->vl53l7cx_set_ranging_mode(VL53L7CX_RANGING_MODE_CONTINUOUS);
    sensor->vl53l7cx_set_ranging_frequency_hz(4);
}


// -------------------------------------------------------
// Time of flight sensors setup fuction
// -------------------------------------------------------
void tofs_setup() {
  mySerCmd.Print((char *) "INFO: Beginning time of flight sensor setup...\r\n");

  Wire.beginTransmission(TOF_LEFT_I2C_ADDRESS);
  if (Wire.endTransmission () == 0) {
    tof_left_state = TOF_STATE_I2CADDRESSASSIGNED;
    mySerCmd.Print((char *) "INFO: Left ToF I2C address is already set\r\n");
  } else {
    tof_left_state = TOF_STATE_DEFAULTI2CADDRESS;
  }

  Wire.beginTransmission(TOF_RIGHT_I2C_ADDRESS);
  if (Wire.endTransmission () == 0) {
    tof_right_state = TOF_STATE_I2CADDRESSASSIGNED;
    mySerCmd.Print((char *) "INFO: Right ToF I2C address is already set\r\n");
  } else {
    tof_right_state = TOF_STATE_DEFAULTI2CADDRESS;
  }

  if (tof_left_state != TOF_STATE_I2CADDRESSASSIGNED && tof_right_state != TOF_STATE_I2CADDRESSASSIGNED) {
    Wire.beginTransmission(TOFS_DEFAULT_I2C_ADDRESS);
    if (Wire.endTransmission () == 0) {
      mySerCmd.Print((char *) "INFO: Detected ToF(s) at their default I2C addresses\r\n");
    } else {
      mySerCmd.Print((char *) "WARNING: No ToF sensors detected!\r\n");
    }
  }

  if (tof_left_state == TOF_STATE_DEFAULTI2CADDRESS) {
    mySerCmd.Print((char *) "INFO: Setting up the left time of flight sensor...\r\n");

    if (tof_right_state == TOF_STATE_DEFAULTI2CADDRESS) {
      mySerCmd.Print((char *) "INFO: Initalizing the right ToF sensor so we can then disable it\r\n");
      sensor_vl53l7cx_right.begin();
      tof_right_state = TOF_STATE_INITIALIZED;
    }

    mySerCmd.Print((char *) "INFO: Disabling the right ToF sensor\r\n");
    sensor_vl53l7cx_right.vl53l7cx_off();
    delay(100);

    mySerCmd.Print((char *) "INFO: Initalizing the left ToF sensor\r\n");
    sensor_vl53l7cx_left.begin();
    tof_left_state = TOF_STATE_INITIALIZED;
    
    mySerCmd.Print((char *) "INFO: Loading firmware into the left ToF sensor (~10 seconds)\r\n");
    sensor_vl53l7cx_left.init_sensor();

    mySerCmd.Print((char *) "INFO: Updating the left ToF sensors I2C address\r\n");
    sensor_vl53l7cx_left.vl53l7cx_set_i2c_address(TOF_LEFT_I2C_ADDRESS << 1); // default: 0x52 (0x29); ours: left: 0x54 (0x2A), right: 0x56 (0x2B)
    tof_left_state = TOF_STATE_I2CADDRESSASSIGNED;

    // turn back on right sensor, we won't init here since we handle the right sensor next
    mySerCmd.Print((char *) "INFO: Enabling the right ToF sensor\r\n");
    sensor_vl53l7cx_right.vl53l7cx_on();
    delay(100);
  } else {
    mySerCmd.Print((char *) "INFO: Re-initializing the left ToF sensor after a reboot...\r\n");
    sensor_vl53l7cx_left.init_sensor(TOF_LEFT_I2C_ADDRESS << 1);
  }

  if (tof_right_state == TOF_STATE_DEFAULTI2CADDRESS || tof_right_state == TOF_STATE_INITIALIZED) {
    mySerCmd.Print((char *) "INFO: Setting up the right time of flight sensor...\r\n");

    if (tof_right_state == TOF_STATE_DEFAULTI2CADDRESS) {
      mySerCmd.Print((char *) "INFO: Initalizing the right ToF sensor\r\n");
      sensor_vl53l7cx_right.begin();
      tof_right_state = TOF_STATE_INITIALIZED;
    }

    mySerCmd.Print((char *) "INFO: Loading firmware into the right ToF sensor (~10 seconds)\r\n");
    sensor_vl53l7cx_right.init_sensor();

    mySerCmd.Print((char *) "INFO: Updating the right ToF sensors I2C address\r\n");
    sensor_vl53l7cx_right.vl53l7cx_set_i2c_address(TOF_RIGHT_I2C_ADDRESS << 1); // default: 0x52 (0x29); ours: left: 0x54 (0x2A), right: 0x56 (0x2B)
    tof_right_state = TOF_STATE_I2CADDRESSASSIGNED;
  } else {
    mySerCmd.Print((char *) "INFO: Re-initializing the right ToF sensor after a reboot...\r\n");
    sensor_vl53l7cx_right.init_sensor(TOF_RIGHT_I2C_ADDRESS << 1);
  }

  if (tof_left_state == TOF_STATE_I2CADDRESSASSIGNED) {
    mySerCmd.Print((char *) "INFO: Configuring settings into the left ToF sensor\r\n");
    set_configuration(&sensor_vl53l7cx_left);
    tof_left_state = TOF_STATE_READY;
  }

  if (tof_right_state == TOF_STATE_I2CADDRESSASSIGNED) {
    mySerCmd.Print((char *) "INFO: Configuring setting into the right ToF sensor\r\n");
    set_configuration(&sensor_vl53l7cx_right);
    tof_right_state = TOF_STATE_READY;
  }

  // Start taking measurments from both ToF sensors
  if (tof_left_state == TOF_STATE_READY && tof_right_state == TOF_STATE_READY) {
    mySerCmd.Print((char *) "INFO: Starting ranging on the left ToF sensor\r\n");
    sensor_vl53l7cx_left.vl53l7cx_start_ranging();
    
    mySerCmd.Print((char *) "INFO: Starting ranging on the right ToF sensor\r\n");
    sensor_vl53l7cx_right.vl53l7cx_start_ranging();
  } else {
    mySerCmd.Print((char *) "ERROR: Time of flight sensor(s) failed setup! Unable to start ranging!\r\n");
  }

  mySerCmd.Print((char *) "INFO: Time of flight sensor setup complete\r\n");
}


// -------------------------------------------------------
// Compare sensor readings to calibrated values
// -------------------------------------------------------
bool compare_result(VL53L7CX *sensor, long calibration_values[]) {
  VL53L7CX_ResultsData Result;
  uint8_t NewDataReady = 0;
  uint8_t status;

  do {
    status = sensor->vl53l7cx_check_data_ready(&NewDataReady);
  } while (!NewDataReady && status == 0);

  if (status != 0 || NewDataReady == 0) {
    return false;
  }

  status = sensor->vl53l7cx_get_ranging_data(&Result);

  if (status != 0) {
    // get_ranging_data did complete successfully
    return false;
  }

  // Evaluate Result
  int8_t i, j, k;
  uint8_t zones_per_line;
  uint8_t number_of_zones = VL53L7CX_RESOLUTION_4X4;
  long distance_value;
  long target_status;
  long compare_value;
  int object_detected = 0;

  zones_per_line = (number_of_zones == 16) ? 4 : 8;

  for (j = 0; j < number_of_zones; j += zones_per_line) {
    for (k = (zones_per_line - 1); k >= 0; k--) {
      distance_value = (long)Result.distance_mm[j+k];
      target_status = (long)Result.target_status[j+k];
      compare_value = distance_value - calibration_values[j+k];
      if ((compare_value <= 0) && (distance_value != 0) && (target_status == 5)) {
        object_detected = 1;
      }
    }
  }

  return (object_detected == 1);
}


// -------------------------------------------------------
// Compare the reading of the left ToF sensor to the
// calibrated values
// -------------------------------------------------------
bool check_left_sensor() {
  if (tof_left_state != TOF_STATE_READY) {
    return false;
  }

  long left_calibration_values[] = {
    220, 275, 271, 397,
    125, 406, 450, 390,
    227, 452, 419, 350,
    250, 387, 364, 312
  };

  return compare_result(&sensor_vl53l7cx_left, left_calibration_values);
}


// -------------------------------------------------------
// Compare the reading of the right ToF sensor to the
// calibrated values
// -------------------------------------------------------
bool check_right_sensor() {
  if (tof_right_state != TOF_STATE_READY) {
    return false;
  }

  long right_calibration_values[] = {
    228, 385, 362, 309,
    233, 430, 400, 338,
    235, 259, 438, 375,
    134, 245, 364, 376
  };
  
  return compare_result(&sensor_vl53l7cx_right, right_calibration_values);
}