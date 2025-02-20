#include <Wire.h>
#include "SerialCmd_Helper.h"
#include "ToFs_Helper.h"

#ifdef TOFS_DEBUG
#define TOFS_VERBOSE_DEBUG(x) mySerCmd.Print(x)
#else
#define TOFS_VERBOSE_DEBUG(x) ;
#endif

extern SerialCmdHelper mySerCmd; // defined in the main sketch

VL53L7CX sensor_vl53l7cx_right(&Wire, 3); // LPn Enable Pin on D3
VL53L7CX sensor_vl53l7cx_left(&Wire, 10); // Requires LPn to be set so using unused pin D10

tofs_state_t tofs_left_state = TOFS_STATE_ABSENT;
tofs_state_t tofs_right_state = TOFS_STATE_ABSENT;

void set_configuration(VL53L7CX *sensor) {
    sensor->vl53l7cx_set_resolution(VL53L7CX_RESOLUTION_4X4); // VL53L7CX_RESOLUTION_4X4, VL53L7CX_RESOLUTION_8X8
    sensor->vl53l7cx_set_target_order(VL53L7CX_TARGET_ORDER_CLOSEST);
    sensor->vl53l7cx_set_ranging_mode(VL53L7CX_RANGING_MODE_CONTINUOUS);
    sensor->vl53l7cx_set_ranging_frequency_hz(4);
}

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

// Public functions

const char* STRING_FOR_TOFS_STATE(tofs_state_t state) {
  switch (state) {
    case TOFS_STATE_ABSENT:
      return "ABSENT";
    case TOFS_STATE_UNCONFIGURED:
      return "UNCONFIGURED";
    case TOFS_STATE_BEGAN:
      return "BEGAN";
    case TOFS_STATE_ASSIGNED:
      return "ASSIGNED";
    case TOFS_STATE_READY:
      return "READY";
    default:
      return "UNKNOWN";
  }
}

void tofs_setup() {
  if (tofs_left_state == TOFS_STATE_UNCONFIGURED) {
    mySerCmd.Print((char *) "INFO: Initializing left sensor on startup...\r\n");

    TOFS_VERBOSE_DEBUG((char *) "      Setting up left sensor with alternate I2C address\r\n");

    if (tofs_right_state == TOFS_STATE_UNCONFIGURED) {
      TOFS_VERBOSE_DEBUG((char *) "      Configuring GPIO for right sensor\r\n");
      sensor_vl53l7cx_right.begin();
      tofs_right_state = TOFS_STATE_BEGAN;
    }

    TOFS_VERBOSE_DEBUG((char *) "      Powering off right sensor\r\n");
    sensor_vl53l7cx_right.vl53l7cx_off();
    delay(100);

    TOFS_VERBOSE_DEBUG((char *) "      Configuring GPIO for left sensor\r\n");
    sensor_vl53l7cx_left.begin();
    tofs_left_state = TOFS_STATE_BEGAN;
    
    TOFS_VERBOSE_DEBUG((char *) "      Loading left sensor firmware\r\n");
    sensor_vl53l7cx_left.init_sensor();
    sensor_vl53l7cx_left.vl53l7cx_set_i2c_address(TOFS_LEFT_I2C_ADDRESS << 1); // default: 0x52 (0x29); ours: left: 0x54 (0x2A), right: 0x56 (0x2B)
    tofs_left_state = TOFS_STATE_ASSIGNED;

    // turn back on right sensor, we won't init here since we handle the right sensor next
    TOFS_VERBOSE_DEBUG((char *) "      Powering on right sensor\r\n");
    sensor_vl53l7cx_right.vl53l7cx_on();
    delay(100);
  } else {
    mySerCmd.Print((char *) "INFO: Re-initializing left sensor after reboot\r\n");
    sensor_vl53l7cx_left.init_sensor(TOFS_LEFT_I2C_ADDRESS << 1);
  }

  if (tofs_right_state == TOFS_STATE_UNCONFIGURED || tofs_right_state == TOFS_STATE_BEGAN) {
    mySerCmd.Print((char *) "INFO: Initializing right sensor on startup...\r\n");

    TOFS_VERBOSE_DEBUG((char *) "      Setting up right sensor with alternate I2C address\r\n");

    TOFS_VERBOSE_DEBUG((char *) "      Powering off left sensor\r\n");
    sensor_vl53l7cx_left.vl53l7cx_off();
    delay(100);

    if (tofs_right_state == TOFS_STATE_UNCONFIGURED) {
      TOFS_VERBOSE_DEBUG((char *) "      Configuring GPIO for right sensor\r\n");
      sensor_vl53l7cx_right.begin();
      tofs_right_state = TOFS_STATE_BEGAN;
    }

    TOFS_VERBOSE_DEBUG((char *) "      Loading right sensor firmware\r\n");
    sensor_vl53l7cx_right.init_sensor();
    sensor_vl53l7cx_right.vl53l7cx_set_i2c_address(TOFS_RIGHT_I2C_ADDRESS << 1); // default: 0x52 (0x29); ours: left: 0x54 (0x2A), right: 0x56 (0x2B)
    tofs_right_state = TOFS_STATE_ASSIGNED;

    // turn back on left sensor, we'll init again here as we already processed the left sensor startup above
    TOFS_VERBOSE_DEBUG((char *) "      Powering on left sensor\r\n");
    sensor_vl53l7cx_left.vl53l7cx_on();
    delay(100);
    TOFS_VERBOSE_DEBUG((char *) "INFO: Re-initializing left sensor after sensor power off\r\n");
    sensor_vl53l7cx_left.init_sensor(TOFS_LEFT_I2C_ADDRESS << 1); // since we powered off the sensor, we must re-init it
  } else {
    mySerCmd.Print((char *) "INFO: Re-initializing right sensor after reboot\r\n");
    sensor_vl53l7cx_right.init_sensor(TOFS_RIGHT_I2C_ADDRESS << 1);
  }

  if (tofs_left_state = TOFS_STATE_ASSIGNED) {
    TOFS_VERBOSE_DEBUG((char *) "INFO: Setting configuration on left sensor\r\n");
    set_configuration(&sensor_vl53l7cx_left);
    tofs_left_state = TOFS_STATE_READY;
  }

  if (tofs_right_state == TOFS_STATE_ASSIGNED) {
    TOFS_VERBOSE_DEBUG((char *) "INFO: Setting configuration on right sensor\r\n");
    set_configuration(&sensor_vl53l7cx_right);
    tofs_right_state = TOFS_STATE_READY;
  }

  // We should ensure we have both sensors for best protection, not just one.
  // This can be changed to an || and we can conditionally start ranging if
  // this isn't the best path.
  //
  // TODO: Determine if one sensor is enough or if both are required.
  // TODO: Should we by default disable movement on the base if the TOFS are misbehaving?
  if (tofs_left_state == TOFS_STATE_READY && tofs_right_state == TOFS_STATE_READY) {
    // Start Measurements
    TOFS_VERBOSE_DEBUG((char *) "INFO: Starting ranging on right sensor\r\n");
    sensor_vl53l7cx_right.vl53l7cx_start_ranging();

    TOFS_VERBOSE_DEBUG((char *) "INFO: Starting ranging on left sensor\r\n");
    sensor_vl53l7cx_left.vl53l7cx_start_ranging();
  } else {
      mySerCmd.Print((char *) "ERROR: Unable to start ranging on both sensors!\r\n");
  }
}

/* --------------------------------  Compare Left Result  -------------------------------*/
bool check_left_sensor() {
  if (tofs_left_state != TOFS_STATE_READY) {
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


/* -------------------------------  Compare Right Result  -------------------------------*/
bool check_right_sensor() {
  if (tofs_right_state != TOFS_STATE_READY) {
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