/*********************************************************************************
Hackerbot Industries, LLC
Created: April 2024
Updated: 2025.01.011

This sketch is written for the "Main Controller" PCBA. It serves several funtions:
  1) Communicate with the SLAM Base Robot
  2) Communicate (if connected) with the Head and Arm
  3) Handle the front time of flight sensors for object detection and avoidance
  4) Get temperature data from the on-board temperature sensor
  5) Handle user code for any user added I2C devices (eg QWIIC or STEMMA sensors)
*********************************************************************************/

#include <SerialCmd.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <vl53l7cx_class.h>

// Main Controller software version
#define VERSION_NUMBER 4

// Set up the serial command processor
SerialCmd mySerCmd(Serial);

// Onboard neopixel setup
Adafruit_NeoPixel onboard_pixel(1, PIN_NEOPIXEL);

// ToF Setup and variables
void compare_left_result(VL53L7CX_ResultsData *Result);
void compare_right_result(VL53L7CX_ResultsData *Result);

VL53L7CX sensor_vl53l7cx_right(&Wire, 3); // LPn Enable Pin on D3
VL53L7CX sensor_vl53l7cx_left(&Wire, 10); // Requires LPn to be set so using unused pin D10
char report[256];

int tofs_attached = 0;
int head_ame_attached = 0;
int head_dyn_attached = 0;
int arm_attached = 0;

// Other defines and variables
byte RxByte;
#define AME_I2C_ADDRESS 90          // Audio Mouth Eyes PCBA I2C address
#define DYN_I2C_ADDRESS 91          // Dynamixel Controller I2C address
#define ARM_I2C_ADDRESS 92          // Arm Controller I2C address


// ------------------- User functions --------------------
void sendOK(void) {
  mySerCmd.Print((char *) "OK\r\n");
}


// --------------- Functions for SerialCmd ---------------
void Send_Ping(void) {
  mySerCmd.Print((char *) "INFO: Main Controller           - ATTACHED\r\n");

  Wire.beginTransmission(75);
  if (Wire.endTransmission () == 0) {
    mySerCmd.Print((char *) "INFO: Temperature Sensor        - ATTACHED\r\n");
  }

  Wire.beginTransmission(41);
  if (Wire.endTransmission () == 0) {
    tofs_attached = 1;
    mySerCmd.Print((char *) "INFO: Time of Flight Sensors    - ATTACHED\r\n");
  }

  Wire.beginTransmission(90); // Head Mouth Eyes PCBA
  if (Wire.endTransmission () == 0) {
    head_ame_attached = 1;
    mySerCmd.Print((char *) "INFO: Audio/Mouth/Eyes PCBA     - ATTACHED\r\n");
  }

  Wire.beginTransmission(91); // Dynamixel Contoller PCBA
  if (Wire.endTransmission () == 0) {
    head_dyn_attached = 1;
    mySerCmd.Print((char *) "INFO: Head Dynamixel Controller - ATTACHED\r\n");
  }

  Wire.beginTransmission(92); // Arm Controller PCBA
  if (Wire.endTransmission () == 0) {
    arm_attached = 1;
    mySerCmd.Print((char *) "INFO: Arm Controller            - ATTACHED\r\n");
  }
  sendOK();
}


// Reports the versions of the boards connected to the Hackerbot
// Example - "VERSION"
void Get_Version(void) {
  mySerCmd.Print((char *) "STATUS: Main Controller (v");
  mySerCmd.Print(VERSION_NUMBER);
  mySerCmd.Print((char *) ".0)\r\n");
  
  if (head_ame_attached == 1) {
    Wire.beginTransmission(AME_I2C_ADDRESS);
    Wire.write(0x02);
    Wire.endTransmission();
    Wire.requestFrom(AME_I2C_ADDRESS, 1);
    while(Wire.available()) {
      RxByte = Wire.read();
    }
    mySerCmd.Print((char *) "STATUS: Audio Mouth Eyes (v");
    mySerCmd.Print(RxByte);
    mySerCmd.Print((char *) ".0)\r\n");
  }

  if (head_dyn_attached == 1) {
    Wire.beginTransmission(DYN_I2C_ADDRESS);
    Wire.write(0x02);
    Wire.endTransmission();
    // I2C RX
    Wire.requestFrom(DYN_I2C_ADDRESS, 1);
    while(Wire.available()) {
      RxByte = Wire.read();
    }
    mySerCmd.Print((char *) "STATUS: Dynamixel Controller (v");
    mySerCmd.Print(RxByte);
    mySerCmd.Print((char *) ".0)\r\n");
  }

  // NEW ARM CODE ADDED
  if (arm_attached == 1) {
    Wire.beginTransmission(ARM_I2C_ADDRESS);
    Wire.write(0x02);
    Wire.endTransmission();
    // I2C RX
    Wire.requestFrom(ARM_I2C_ADDRESS, 1);
    while(Wire.available()) {
      RxByte = Wire.read();
    }
    mySerCmd.Print((char *) "STATUS: Arm Controller (v");
    mySerCmd.Print(RxByte);
    mySerCmd.Print((char *) ".0)\r\n");
  }
  // END NEW ARM CODE ADDED

  sendOK();
}


// Initializes the connection between the Arduino on the Main Controller with the SLAM vacuum base robot. This must be run once after a power cycle before any other commands can be sent
// Example - "INIT"
void Send_Handshake(void) {
  byte handshake1_frame[] = {
    0x55, 0xAA, // HEADER_HI, HEADER_LOW
    0x02, // CTRL_ID
    0x00, 0x51, // LEN_HI, LEN_LOW
    0x01, // PACKET_ID
    0x4f, // PACKET_LEN
    0x00, // step
    0x50, 0x43, 0x5F, 0x54, 0x45, 0x53, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00,// model "PC_TEST"
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // core_version
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // chassis_version
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // mac_addr
    0x00, 0x00, 0x00, 0x00, // ip
    0xA5, // use_uart 0xa5
    0x00, // rssi
    0x00, // noise_level
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // reserved
    0x89, 0xab // CRC_HI, CRC_LOW
  };

  byte content[88];
  mySerCmd.Print((char *) "Sending handshake1_frame\r\n");
  Serial1.write(handshake1_frame, sizeof(handshake1_frame));
  for(int i = 0; i < 88; i++) {
    while(!Serial1.available());
    byte incomingByte = Serial1.read();
    String incomingHex = String(incomingByte, HEX);
    incomingHex.toUpperCase();
    content[i] = (byte)strtoul(incomingHex.c_str(), NULL, 16);
    mySerCmd.Print(incomingHex);
    mySerCmd.Print((char *) " ");
  }

  content[2] = 0x02;
  // Calculate the CRC of the frame (clipping off the first 2 and last 2 bytes sent to the crc function)
  uint8_t crcHex[2];
  uint16_t crc = crc16(&content[2], sizeof(content)-4);
  memcpy(crcHex, &crc, sizeof(crcHex));
  content[sizeof(content)-2] = crcHex[1];
  content[sizeof(content)-1] = crcHex[0];

  mySerCmd.Print((char *) "Sending handshake2_frame\r\n");
  //Serial1.write(handshake2_frame, sizeof(handshake2_frame));
  Serial1.write(content, sizeof(content));
  sendOK();
}


// Gets a list of all of the maps stored on the robot
// Example - "GETML"
void Get_MapList(void) {
  byte get_map_list_frame[] = {
    0x55, 0xAA, // HEADER_HI, HEADER_LOW
    0x02, // CTRL_ID
    0x00, 0x17, // LEN_HI, LEN_LOW
    0x20, // PACKET_ID
    0x15, // PACKET_LEN
    0x00, // map_num
    0x00, 0x00, 0x00, 0x00, // map_id_1
    0x00, 0x00, 0x00, 0x00, // map_id_2
    0x00, 0x00, 0x00, 0x00, // map_id_3
    0x00, 0x00, 0x00, 0x00, // map_id_4
    0x00, 0x00, 0x00, 0x00, // map_id_5
    0x9A, 0x8A // CRC_HI, CRC_LOW
  };
  Serial1.write(get_map_list_frame, sizeof(get_map_list_frame));

  sendOK();
}


// Set mode to enter. This moves the robot off the dock and starts the LiDAR
// Example - "ENTER"
void Send_Enter(void) {
  byte behavior_control_frame[] = {
    0x55, 0xAA, // HEADER_HI, HEADER_LOW
    0x02, // CTRL_ID
    0x00, 0x44, // LEN_HI, LEN_LOW
    0x22, // PACKET_ID
    0x42, // PACKET_LEN
    0x00, // bid
    0x00, // status
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // reserved[32];
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // reserved[32];
    0xB0, 0xA5 // CRC_HI, CRC_LOW
  };
  Serial1.write(behavior_control_frame, sizeof(behavior_control_frame));

  sendOK();
}


// Go to pose command
// Parameters
// float: x, float: y, float: angle, float: speed
// Example - "GOTO,0.5,0.5,0,10"
void Send_Goto(void) {
  // Handle command parameters
  float xParam = atof(mySerCmd.ReadNext());
  float yParam = atof(mySerCmd.ReadNext());
  float aParam = atof(mySerCmd.ReadNext());
  float sParam = atof(mySerCmd.ReadNext());

  uint8_t xParamHex[4];
  uint8_t yParamHex[4];
  uint8_t aParamHex[4];
  uint8_t sParamHex[4];

  memcpy(xParamHex, &xParam, sizeof(xParamHex));
  memcpy(yParamHex, &yParam, sizeof(yParamHex));
  memcpy(aParamHex, &aParam, sizeof(aParamHex));
  memcpy(sParamHex, &sParam, sizeof(sParamHex));

  // Create the frame
  byte behavior_control_goto_frame[] = {
    0x55, 0xAA, // HEADER_HI, HEADER_LOW
    0x02, // CTRL_ID
    0x00, 0x44, // LEN_HI, LEN_LOW
    0x22, // PACKET_ID
    0x42, // PACKET_LEN
    0x04, // bid
    0x00, // status
    xParamHex[0], xParamHex[1], xParamHex[2], xParamHex[3], // x
    yParamHex[0], yParamHex[1], yParamHex[2], yParamHex[3], // y
    aParamHex[0], aParamHex[1], aParamHex[2], aParamHex[3], // angle
    sParamHex[0], sParamHex[1], sParamHex[2], sParamHex[3], // speed
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // reserved[48];
    0x00, 0x00 // CRC_HI, CRC_LOW
  };

   // Calculate the CRC of the frame (clipping off the first 2 and last 2 bytes sent to the crc function)
  uint8_t crcHex[2];
  uint16_t crc = crc16(&behavior_control_goto_frame[2], sizeof(behavior_control_goto_frame)-4);
  memcpy(crcHex, &crc, sizeof(crcHex));
  behavior_control_goto_frame[sizeof(behavior_control_goto_frame)-2] = crcHex[1];
  behavior_control_goto_frame[sizeof(behavior_control_goto_frame)-1] = crcHex[0];

  // Display the frame to be sent
  for(int i = 0; i < sizeof(behavior_control_goto_frame); i++) {
    String incomingHex = String(behavior_control_goto_frame[i], HEX);
    incomingHex.toUpperCase();
    mySerCmd.Print(incomingHex);
    mySerCmd.Print((char *) " ");
  }
  mySerCmd.Print((char *) "\r\n");

  // Send the frame
  Serial1.write(behavior_control_goto_frame, sizeof(behavior_control_goto_frame));

  sendOK();
}


// Go to dock command
// Example - "DOCK"
void Send_Dock(void) {
  // Create the frame
  byte behavior_control_dock_frame[] = {
    0x55, 0xAA, // HEADER_HI, HEADER_LOW
    0x02, // CTRL_ID
    0x00, 0x44, // LEN_HI, LEN_LOW
    0x22, // PACKET_ID
    0x42, // PACKET_LEN
    0x06, // bid
    0x00, // status
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // reserved[32];
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // reserved[32];
    0x00, 0x00 // CRC_HI, CRC_LOW
  };

   // Calculate the CRC of the frame (clipping off the first 2 and last 2 bytes sent to the crc function)
  uint8_t crcHex[2];
  uint16_t crc = crc16(&behavior_control_dock_frame[2], sizeof(behavior_control_dock_frame)-4);
  memcpy(crcHex, &crc, sizeof(crcHex));
  behavior_control_dock_frame[sizeof(behavior_control_dock_frame)-2] = crcHex[1];
  behavior_control_dock_frame[sizeof(behavior_control_dock_frame)-1] = crcHex[0];

  // Display the frame to be sent
  for(int i = 0; i < sizeof(behavior_control_dock_frame); i++) {
    String incomingHex = String(behavior_control_dock_frame[i], HEX);
    incomingHex.toUpperCase();
    mySerCmd.Print(incomingHex);
    mySerCmd.Print((char *) " ");
  }
  mySerCmd.Print((char *) "\r\n");

  // Send the frame
  Serial1.write(behavior_control_dock_frame, sizeof(behavior_control_dock_frame));

  sendOK();
}


// Simulated bumper command
// Parameters
// bool: left, bool: right
// Example - "BUMP,1,0"
void Send_Bump(void) {
  // Create the frame
  byte simbumpersignal_frame[] = {
    0x55, 0xAA, // HEADER_HI, HEADER_LOW
    0x02, // CTRL_ID
    0x00, 0x03, // LEN_HI, LEN_LOW
    0x24, // PACKET_ID
    0x01, // PACKET_LEN
    0x01, // binary: 0, 0, 0, 0, 0, 0, left, right
    0x00, 0x00 // CRC_HI, CRC_LOW
  };

   // Calculate the CRC of the frame (clipping off the first 2 and last 2 bytes sent to the crc function)
  uint8_t crcHex[2];
  uint16_t crc = crc16(&simbumpersignal_frame[2], sizeof(simbumpersignal_frame)-4);
  memcpy(crcHex, &crc, sizeof(crcHex));
  simbumpersignal_frame[sizeof(simbumpersignal_frame)-2] = crcHex[1];
  simbumpersignal_frame[sizeof(simbumpersignal_frame)-1] = crcHex[0];

  // Display the frame to be sent
  for(int i = 0; i < sizeof(simbumpersignal_frame); i++) {
    String incomingHex = String(simbumpersignal_frame[i], HEX);
    incomingHex.toUpperCase();
    mySerCmd.Print(incomingHex);
    mySerCmd.Print((char *) " ");
  }
  mySerCmd.Print((char *) "\r\n");

  // Send the frame
  Serial1.write(simbumpersignal_frame, sizeof(simbumpersignal_frame));

  sendOK();
}


// Wheel motor packet
// Parameters
// int16_t: linear_velocity, int16_t: angular_velocity
// Example - "MOTOR,1,0"
void Send_Motor(void) {
  // Create the frame
  byte motor_frame[] = {
    0x55, 0xAA, // HEADER_HI, HEADER_LOW
    0x02, // CTRL_ID
    0x00, 0x06, // LEN_HI, LEN_LOW
    0x09, // PACKET_ID
    0x04, // PACKET_LEN
    0x9C, 0xFF, // linear velocity
    0x00, 0x00, // angular velocity
    0x00, 0x00 // CRC_HI, CRC_LOW
  };

   // Calculate the CRC of the frame (clipping off the first 2 and last 2 bytes sent to the crc function)
  uint8_t crcHex[2];
  uint16_t crc = crc16(&motor_frame[2], sizeof(motor_frame)-4);
  memcpy(crcHex, &crc, sizeof(crcHex));
  motor_frame[sizeof(motor_frame)-2] = crcHex[1];
  motor_frame[sizeof(motor_frame)-1] = crcHex[0];

  // Display the frame to be sent
  for(int i = 0; i < sizeof(motor_frame); i++) {
    String incomingHex = String(motor_frame[i], HEX);
    incomingHex.toUpperCase();
    mySerCmd.Print(incomingHex);
    mySerCmd.Print((char *) " ");
  }
  mySerCmd.Print((char *) "\r\n");

  // Send the frame
  Serial1.write(motor_frame, sizeof(motor_frame));

  sendOK();
}


// -------------------------------------------------------
// Head Functions
// -------------------------------------------------------
void set_IDLE(void) {
  char * sParam;
  sParam = mySerCmd.ReadNext();

  if (head_ame_attached == 0) {
    mySerCmd.Print((char *) "ERROR: Dynamixel controller not attached\r\n");
    return;
  }
 
  if (sParam == NULL) {
    mySerCmd.Print((char *) "ERROR: Missing idle parameter\r\n");
    return;
  }

  if (strtoul(sParam, NULL, 10) == 0) {
    Wire.beginTransmission(DYN_I2C_ADDRESS);
    Wire.write(0x08);
    Wire.write(0x00);
    Wire.endTransmission();
    mySerCmd.Print((char *) "STATUS: Head idle mode disabled\r\n");
  } else {
    Wire.beginTransmission(DYN_I2C_ADDRESS);
    Wire.write(0x08);
    Wire.write(0x01);
    Wire.endTransmission();
    mySerCmd.Print((char *) "STATUS: Head idle mode enabled\r\n");
  }

  sendOK();
}

void set_LOOK(void) {
  float turnParam = atof(mySerCmd.ReadNext());
  float vertParam = atof(mySerCmd.ReadNext());
  float speedParam = atof(mySerCmd.ReadNext());

  if (head_ame_attached == 0) {
    mySerCmd.Print((char *) "ERROR: Dynamixel controller not attached\r\n");
    return;
  }
 
  if ((turnParam == NULL) || (vertParam == NULL) || (speedParam == NULL)) {
    mySerCmd.Print((char *) "ERROR: Missing parameter\r\n");
    return;
  }

  if (turnParam < 100.0) {
    turnParam = 100.0;
  } else if (turnParam > 260.0) {
    turnParam = 260.0;
  }

  if (vertParam < 150.0) {
    vertParam = 150.0;
  } else if (vertParam > 250.0) {
    vertParam = 250.0;
  }

  if (speedParam < 6) {
    speedParam = 6;
  } else if (speedParam > 70) {
    speedParam = 70;
  }

  mySerCmd.Print((char *) "STATUS: Looking to position turn: ");
  mySerCmd.Print(turnParam);
  mySerCmd.Print((char *) ", vert: ");
  mySerCmd.Print(vertParam);
  mySerCmd.Print((char *) ", at speed: ");
  mySerCmd.Print((int)(speedParam));
  mySerCmd.Print((char *) "\r\n");

  uint16_t turnParam16 = (uint16_t)(turnParam * 10);
  uint16_t vertParam16 = (uint16_t)(vertParam * 10);
  uint8_t speedParam8 = (uint8_t)(speedParam);

  Wire.beginTransmission(DYN_I2C_ADDRESS);
  Wire.write(0x09);
  Wire.write(highByte(turnParam16)); // Yaw H
  Wire.write(lowByte(turnParam16)); // YAW L
  Wire.write(highByte(vertParam16)); // Pitch H
  Wire.write(lowByte(vertParam16)); // Pitch L
  Wire.write(speedParam8);
  Wire.endTransmission();

  sendOK();
}

// NEW ARM CODE ADDED
// ACAL
void run_CALIBRATION(void) {
  mySerCmd.Print((char *) "INFO: Calibrating the gripper\r\n");

  Wire.beginTransmission(ARM_I2C_ADDRESS);
  Wire.write(0x20);
  Wire.endTransmission();

  sendOK();
}

// AOPEN
void run_OPEN(void) {
  mySerCmd.Print((char *) "INFO: Opening the gripper\r\n");

  Wire.beginTransmission(ARM_I2C_ADDRESS);
  Wire.write(0x21);
  Wire.endTransmission();
  
  sendOK();
}

// ACLOSE
void run_CLOSE(void) {
  mySerCmd.Print((char *) "INFO: Closing the gripper\r\n");

  Wire.beginTransmission(ARM_I2C_ADDRESS);
  Wire.write(0x22);
  Wire.endTransmission();
  
  sendOK();
}

// AANGLE
void set_ANGLE(void) {
  float jointParam = atof(mySerCmd.ReadNext());
  float angleParam = atof(mySerCmd.ReadNext());
  float speedParam = atof(mySerCmd.ReadNext());

  if (speedParam == NULL) {
    mySerCmd.Print((char *) "ERROR: Missing parameter\r\n");
    return;
  }

  if (jointParam < 0) {
    jointParam = 0;
  } else if (jointParam > 6) {
    jointParam = 6;
  }

  if (angleParam < -165.0) {
    angleParam = -165.0;
  } else if (angleParam > 165.0) {
    angleParam = 165.0;
  }

  if (speedParam < 0) {
    speedParam = 0;
  } else if (speedParam > 100) {
    speedParam = 100;
  }

  mySerCmd.Print((char *) "STATUS: Setting the angle of joint ");
  mySerCmd.Print((int)jointParam);
  mySerCmd.Print((char *) " to ");
  mySerCmd.Print(angleParam);
  mySerCmd.Print((char *) " degrees at speed ");
  mySerCmd.Print((int)speedParam);
  mySerCmd.Print((char *) "\r\n");

  uint8_t jointParam8 = (uint8_t)(jointParam);
  uint16_t angleParam16 = (uint16_t)((angleParam + 165.0) * 10);
  uint8_t speedParam8 = (uint8_t)(speedParam);

  Wire.beginTransmission(ARM_I2C_ADDRESS);
  Wire.write(0x25);
  Wire.write(jointParam8);
  Wire.write(highByte(angleParam16)); // Angle H
  Wire.write(lowByte(angleParam16)); // Angle L
  Wire.write(speedParam8);
  Wire.endTransmission();

  sendOK();
}

// AANGLES
void set_ANGLES(void) {
  float joint1Param = atof(mySerCmd.ReadNext());
  float joint2Param = atof(mySerCmd.ReadNext());
  float joint3Param = atof(mySerCmd.ReadNext());
  float joint4Param = atof(mySerCmd.ReadNext());
  float joint5Param = atof(mySerCmd.ReadNext());
  float joint6Param = atof(mySerCmd.ReadNext());
  float speedParam = atof(mySerCmd.ReadNext());

  if (speedParam == NULL) {
    mySerCmd.Print((char *) "ERROR: Missing parameter\r\n");
    return;
  }

  if (joint1Param < -165.0)
    joint1Param = 165.0;
  else if (joint1Param > 165.0)
    joint1Param = 165.0;

  if (joint2Param < -165.0)
    joint2Param = 165.0;
  else if (joint2Param > 165.0)
    joint2Param = 165.0;

  if (joint3Param < -165.0)
    joint3Param = 165.0;
  else if (joint3Param > 165.0)
    joint3Param = 165.0;

  if (joint4Param < -165.0)
    joint4Param = 165.0;
  else if (joint4Param > 165.0)
    joint4Param = 165.0;

  if (joint5Param < -165.0)
    joint5Param = 165.0;
  else if (joint5Param > 165.0)
    joint5Param = 165.0;

  if (joint6Param < -175.0)
    joint6Param = 175.0;
  else if (joint6Param > 175.0)
    joint6Param = 175.0;

  if (speedParam < 0)
    speedParam = 0;
  else if (speedParam > 100)
    speedParam = 100;

  Angles angles = {joint1Param, joint2Param, joint3Param, joint4Param, joint5Param, joint6Param};

  mySerCmd.Print((char *) "STATUS: Setting the angle of the joints to (1) ");
  mySerCmd.Print(joint1Param);
  mySerCmd.Print((char *) ", (2) ");
  mySerCmd.Print(joint2Param);
  mySerCmd.Print((char *) ", (3) ");
  mySerCmd.Print(joint3Param);
  mySerCmd.Print((char *) ", (4) ");
  mySerCmd.Print(joint4Param);
  mySerCmd.Print((char *) ", (5) ");
  mySerCmd.Print(joint5Param);
  mySerCmd.Print((char *) ", (6) ");
  mySerCmd.Print(joint6Param);
  mySerCmd.Print((char *) " degrees at speed ");
  mySerCmd.Print((int)speedParam);
  mySerCmd.Print((char *) "\r\n");

  uint16_t joint1Param16 = (uint16_t)((joint1Param + 165.0) * 10);
  uint16_t joint2Param16 = (uint16_t)((joint2Param + 165.0) * 10);
  uint16_t joint3Param16 = (uint16_t)((joint3Param + 165.0) * 10);
  uint16_t joint4Param16 = (uint16_t)((joint4Param + 165.0) * 10);
  uint16_t joint5Param16 = (uint16_t)((joint5Param + 165.0) * 10);
  uint16_t joint6Param16 = (uint16_t)((joint6Param + 175.0) * 10);
  uint8_t speedParam8 = (uint8_t)(speedParam);

  Wire.beginTransmission(ARM_I2C_ADDRESS);
  Wire.write(0x26);
  Wire.write(highByte(joint1Param16)); // Joint 1 Angle H
  Wire.write(lowByte(joint1Param16)); // Joint 1 Angle L
  Wire.write(highByte(joint2Param16)); // Joint 2 Angle H
  Wire.write(lowByte(joint2Param16)); // Joint 2 Angle L
  Wire.write(highByte(joint3Param16)); // Joint 3 Angle H
  Wire.write(lowByte(joint3Param16)); // Joint 3 Angle L
  Wire.write(highByte(joint4Param16)); // Joint 4 Angle H
  Wire.write(lowByte(joint4Param16)); // Joint 4 Angle L
  Wire.write(highByte(joint5Param16)); // Joint 5 Angle H
  Wire.write(lowByte(joint5Param16)); // Joint 5 Angle L
  Wire.write(highByte(joint6Param16)); // Joint 6 Angle H
  Wire.write(lowByte(joint6Param16)); // Joint 6 Angle L
  Wire.write(speedParam8);
  Wire.endTransmission();

  sendOK();
}
// END ARM CODE ADDED


/* --------------------------------  Compare Left Result  -------------------------------*/
void compare_left_result(VL53L7CX_ResultsData *Result) {
  int8_t i, j, k;
  uint8_t zones_per_line;
  uint8_t number_of_zones = VL53L7CX_RESOLUTION_4X4;
  long left_calibration_values[] = {220, 275, 271, 397, 125, 406, 450, 390, 227, 452, 419, 350, 250, 387, 364, 312};
  long distance_value;
  long target_status;
  long compare_value;
  int object_detected = 0;

  zones_per_line = (number_of_zones == 16) ? 4 : 8;

  for (j = 0; j < number_of_zones; j += zones_per_line) {
    for (k = (zones_per_line - 1); k >= 0; k--) {
      distance_value = (long)Result->distance_mm[j+k];
      target_status = (long)Result->target_status[j+k];
      compare_value = distance_value - left_calibration_values[j+k];
      if ((compare_value <= 0) && (distance_value != 0) && (target_status == 5)) {
        object_detected = 1;
      }
    }
  }

  if (object_detected == 1) {
    mySerCmd.Print((char *) "STATUS: Left Object Detected!\r\n");
    Send_Bump();
  }
}


/* -------------------------------  Compare Right Result  -------------------------------*/
void compare_right_result(VL53L7CX_ResultsData *Result) {
  int8_t i, j, k;
  uint8_t zones_per_line;
  uint8_t number_of_zones = VL53L7CX_RESOLUTION_4X4;
  long right_calibration_values[] = {228, 385, 362, 309, 233, 430, 400, 338, 235, 259, 438, 375, 134, 245, 364, 376};
  long distance_value;
  long target_status;
  long compare_value;
  int object_detected = 0;

  zones_per_line = (number_of_zones == 16) ? 4 : 8;

  for (j = 0; j < number_of_zones; j += zones_per_line) {
    for (k = (zones_per_line - 1); k >= 0; k--) {
      distance_value = (long)Result->distance_mm[j+k];
      target_status = (long)Result->target_status[j+k];
      compare_value = distance_value - right_calibration_values[j+k];
      if ((compare_value <= 0) && (distance_value != 0) && (target_status == 5)) {
        object_detected = 1;
      }
    }
  }

  if (object_detected == 1) {
    mySerCmd.Print((char *) "STATUS: Right Object Detected!\r\n");
    Send_Bump();
  }
}


// ----------------------- setup() -----------------------
void setup() {
  unsigned long serialTimout = millis();

  Serial.begin(115200);
  while(!Serial && millis() - serialTimout <= 5000);

  Serial1.begin(230400);

  delay(1000);

  mySerCmd.Print((char *) "INFO: Initalizing application...\r\n");

  onboard_pixel.begin();
  onboard_pixel.setPixelColor(0, onboard_pixel.Color(0, 10, 0));
  onboard_pixel.show();

  // Command Setup
  mySerCmd.AddCmd("PING", SERIALCMD_FROMALL, Send_Ping);
  mySerCmd.AddCmd("VERSION", SERIALCMD_FROMALL, Get_Version);
  mySerCmd.AddCmd("INIT", SERIALCMD_FROMALL, Send_Handshake);
  mySerCmd.AddCmd("ENTER", SERIALCMD_FROMALL, Send_Enter);
  mySerCmd.AddCmd("GOTO", SERIALCMD_FROMALL, Send_Goto);
  mySerCmd.AddCmd("DOCK", SERIALCMD_FROMALL, Send_Dock);
  mySerCmd.AddCmd("BUMP", SERIALCMD_FROMALL, Send_Bump);
  mySerCmd.AddCmd("MOTOR", SERIALCMD_FROMALL, Send_Motor);  
  //mySerCmd.AddCmd("GETML", SERIALCMD_FROMALL, Get_MapList);

  // Head Commands
  mySerCmd.AddCmd("HIDLE", SERIALCMD_FROMALL, set_IDLE);
  mySerCmd.AddCmd("HLOOK", SERIALCMD_FROMALL, set_LOOK);

  // NEW ARM CODE ADDED
  // Arm Commands
  mySerCmd.AddCmd("ACAL", SERIALCMD_FROMALL, run_CALIBRATION);
  mySerCmd.AddCmd("AOPEN", SERIALCMD_FROMALL, set_OPEN);
  mySerCmd.AddCmd("ACLOSE", SERIALCMD_FROMALL, set_CLOSE);
  mySerCmd.AddCmd("AANGLE", SERIALCMD_FROMALL, set_ANGLE);
  mySerCmd.AddCmd("AANGLES", SERIALCMD_FROMALL, set_ANGLES);
  // END NEW ARM CODE ADDED

  // Initialize I2C bus
  Wire.begin();

  // Scan for I2C devices (prevents the application from locking up if an accessory is missing)
  Wire.beginTransmission(75);
  if (Wire.endTransmission () == 0) {
    mySerCmd.Print((char *) "STATUS: Temperature Sensor Attached\r\n");
  }

  Wire.beginTransmission(41);
  if (Wire.endTransmission () == 0) {
    tofs_attached = 1;
    mySerCmd.Print((char *) "STATUS: Time of Flight Sensors Attached\r\n");
  }

  Wire.beginTransmission(90); // Head Mouth Eyes PCBA
  if (Wire.endTransmission () == 0) {
    head_ame_attached = 1;
    mySerCmd.Print((char *) "STATUS: Hackerbot Head Audio/Mouth/Eyes PCBA Attached\r\n");
  }

  Wire.beginTransmission(91); // Dynamixel Contoller PCBA
  if (Wire.endTransmission () == 0) {
    head_dyn_attached = 1;
    mySerCmd.Print((char *) "STATUS: Hackerbot Head Dynamixel Controller Attached\r\n");
  }

  Wire.beginTransmission(92); // Arm Controller PCBA
  if (Wire.endTransmission () == 0) {
    arm_attached = 1;
    mySerCmd.Print((char *) "STATUS: Hackerbot Arm Controller Attached\r\n");
  }
  
  // ToF Setup and Configuration (if attached)
  if (tofs_attached) {
    mySerCmd.Print((char *) "INFO: Downloading sensor firmware and itializing settings...\r\n");

    // Configure VL53L7CX component.
    sensor_vl53l7cx_right.begin();
    mySerCmd.Print((char *) "INFO: Disabling right sensor\r\n");
    sensor_vl53l7cx_right.vl53l7cx_off();

    delay(100);

    sensor_vl53l7cx_left.begin();
    
    mySerCmd.Print((char *) "INFO: Loading left sensor firmware\r\n");
    sensor_vl53l7cx_left.init_sensor();

    mySerCmd.Print((char *) "INFO: Loading left sensor settings\r\n");
    sensor_vl53l7cx_left.vl53l7cx_set_resolution(VL53L7CX_RESOLUTION_4X4); // VL53L7CX_RESOLUTION_4X4, VL53L7CX_RESOLUTION_8X8
    sensor_vl53l7cx_left.vl53l7cx_set_target_order(VL53L7CX_TARGET_ORDER_CLOSEST);
    sensor_vl53l7cx_left.vl53l7cx_set_ranging_mode(VL53L7CX_RANGING_MODE_CONTINUOUS);
    sensor_vl53l7cx_left.vl53l7cx_set_ranging_frequency_hz(4);
    sensor_vl53l7cx_left.vl53l7cx_set_i2c_address(0x54); // 0x52 (0x29), 0x54 (0x2A)

    // Configure VL53L7CX component.
    mySerCmd.Print((char *) "INFO: Enabling right sensor\r\n");
    sensor_vl53l7cx_right.vl53l7cx_on();

    mySerCmd.Print((char *) "INFO: Loading right sensor firmware\r\n");
    sensor_vl53l7cx_right.init_sensor();

    mySerCmd.Print((char *) "INFO: Loading right sensor settings\r\n");
    sensor_vl53l7cx_right.vl53l7cx_set_resolution(VL53L7CX_RESOLUTION_4X4); // VL53L7CX_RESOLUTION_4X4, VL53L7CX_RESOLUTION_8X8
    sensor_vl53l7cx_right.vl53l7cx_set_target_order(VL53L7CX_TARGET_ORDER_CLOSEST);
    sensor_vl53l7cx_right.vl53l7cx_set_ranging_mode(VL53L7CX_RANGING_MODE_CONTINUOUS);
    sensor_vl53l7cx_right.vl53l7cx_set_ranging_frequency_hz(4);

    mySerCmd.Print((char *) "INFO: Initialization of serial port and sensor is complete. Start ranging.\r\n");

    // Start Measurements
    sensor_vl53l7cx_right.vl53l7cx_start_ranging();
    sensor_vl53l7cx_left.vl53l7cx_start_ranging();
  }

  onboard_pixel.setPixelColor(0, onboard_pixel.Color(0, 0, 10));
  onboard_pixel.show();
  mySerCmd.Print((char *) "INFO: Starting application...\r\n");
}


// ----------------------- loop() ------------------------
void loop() {
  byte incomingByte;
  byte incomingPacket[100];
  int incomingPacketLen;
  byte lenByte;
  int8_t ret;
  
   // Read ToF sensor values and send a simulated bump command if an object is too close (only run if tof sensors are attached)
   if (tofs_attached) {
    VL53L7CX_ResultsData Results;
    uint8_t NewDataReadyRight = 0;
    uint8_t statusRight;
    uint8_t NewDataReadyLeft = 0;
    uint8_t statusLeft;

    do {
      statusRight = sensor_vl53l7cx_right.vl53l7cx_check_data_ready(&NewDataReadyRight);
    } while (!NewDataReadyRight);

    if ((!statusRight) && (NewDataReadyRight != 0)) {
      statusRight = sensor_vl53l7cx_right.vl53l7cx_get_ranging_data(&Results);
      compare_right_result(&Results);
    }

    do {
      statusLeft = sensor_vl53l7cx_left.vl53l7cx_check_data_ready(&NewDataReadyLeft);
    } while (!NewDataReadyLeft);

    if ((!statusLeft) && (NewDataReadyLeft != 0)) {
      statusLeft = sensor_vl53l7cx_left.vl53l7cx_get_ranging_data(&Results);
      compare_left_result(&Results);
    }
  }

  // Check for incoming serial commands
  ret = mySerCmd.ReadSer();
  if (ret == 0) {
    mySerCmd.Print((char *) "ERROR: Urecognized command\r\n");
  }

  // Check for data coming from the SLAM base robot
  if (Serial1.available()) {
    incomingByte = Serial1.read();
    incomingPacket[incomingPacketLen] = incomingByte;
    incomingPacketLen++;
    if (incomingByte == 0x55) {
      while(!Serial1.available());
      incomingByte = Serial1.read();
      incomingPacket[incomingPacketLen] = incomingByte;
      incomingPacketLen++;
      if (incomingByte == 0xAA) {
        while(!Serial1.available());
        incomingByte = Serial1.read();
        incomingPacket[incomingPacketLen] = incomingByte;
        incomingPacketLen++;
        while(!Serial1.available());
        incomingByte = Serial1.read();
        incomingPacket[incomingPacketLen] = incomingByte;
        incomingPacketLen++;
        while(!Serial1.available());
        lenByte = Serial1.read();
        incomingPacket[incomingPacketLen] = lenByte;
        incomingPacketLen++;
        for (int i = 0; i < (lenByte+2); i++) {
          while(!Serial1.available());
          incomingByte = Serial1.read();
          incomingPacket[incomingPacketLen] = incomingByte;
          incomingPacketLen++;
        }

        if ((incomingPacket[5] != 0x02) && ((incomingPacket[5] != 0x23))) {
          for(int i = 0; i < incomingPacketLen; i++) {
            String incomingHex = String(incomingPacket[i], HEX);
            incomingHex.toUpperCase();
            mySerCmd.Print(incomingHex);
            mySerCmd.Print((char *) " ");
          }
          mySerCmd.Print((char *) "\r\n");
        }
      }
    }
  }
}



// CRC-16/XMODEM https://crccalc.com
static unsigned short const crc16_table[256] = {
0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};


static uint16_t crc16(uint8_t* data, uint16_t length) {
  uint16_t crc = 0;
  while (length--) {
      crc = (crc << 8) ^ crc16_table[((crc >> 8) ^ *data++) & 0x00FF];
  }

  return crc;
}