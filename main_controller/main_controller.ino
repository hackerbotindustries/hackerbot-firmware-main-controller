/*********************************************************************************
Hackerbot Industries, LLC
Created: April 2024
Updated: 2025.02.20

This sketch is written for the "Main Controller" PCBA. It serves several funtions:
  1) Communicate with the SLAM Base Robot
  2) Communicate (if connected) with the Head and Arm
  3) Handle the front time of flight sensors for object detection and avoidance
  4) Get temperature data from the on-board temperature sensor
  5) Handle user code for any user added I2C devices (eg QWIIC or STEMMA sensors)
*********************************************************************************/

#include <Wire.h>
#include <SerialCmd.h>
#include <Adafruit_NeoPixel.h>
#include <vl53l7cx_class.h>

#include "Hackerbot_Shared.h"
#include "SerialCmd_Helper.h"
#include "ToFs_Helper.h"
#include "SLAM_Base_Frames.h"

// Main Controller software version
#define VERSION_NUMBER 7

// Set up the serial command processor
SerialCmdHelper mySerCmd(Serial);

// Onboard neopixel setup
Adafruit_NeoPixel onboard_pixel(1, PIN_NEOPIXEL);

// Other defines and variables
#define TEMP_SENSOR_I2C_ADDRESS 0x4B
byte RxByte;

bool head_ame_attached = false;
bool head_dyn_attached = false;
bool arm_attached = false;
bool tofs_attached = false;
bool temperature_sensor_attached = false;

// Function prototypes
void Get_Packet(byte response_packetid = 0x00, byte response_frame[] = nullptr, int sizeOfResponseFrame = 0);
void Get_File_Transfer_Packet(byte request_ctrlid = 0x00);


// -------------------------------------------------------
// setup()
// -------------------------------------------------------
void setup() {
  unsigned long serialTimout = millis();

  Serial.begin(115200);
  while(!Serial && millis() - serialTimout <= 5000);

  Serial1.begin(230400);

  delay(1000);

  mySerCmd.Print((char *) "INFO: Initalizing application...\r\n");
  mySerCmd.Print((char *) "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\r\n");

  onboard_pixel.begin();
  onboard_pixel.setPixelColor(0, onboard_pixel.Color(0, 0, 10));
  onboard_pixel.show();

  // Command Setup
  mySerCmd.AddCmd("PING", SERIALCMD_FROMALL, Send_Ping);
  mySerCmd.AddCmd("VERSION", SERIALCMD_FROMALL, Get_Version);
  mySerCmd.AddCmd("INIT", SERIALCMD_FROMALL, Send_Handshake);
  mySerCmd.AddCmd("MODE", SERIALCMD_FROMALL, Send_Mode);
  mySerCmd.AddCmd("ENTER", SERIALCMD_FROMALL, Send_Enter);
  mySerCmd.AddCmd("QUICKMAP", SERIALCMD_FROMALL, Send_QuickMap);
  mySerCmd.AddCmd("GOTO", SERIALCMD_FROMALL, Send_Goto);
  mySerCmd.AddCmd("DOCK", SERIALCMD_FROMALL, Send_Dock);
  mySerCmd.AddCmd("STOP", SERIALCMD_FROMALL, Send_Stop);
  mySerCmd.AddCmd("BUMP", SERIALCMD_FROMALL, Send_Bump);
  mySerCmd.AddCmd("MOTOR", SERIALCMD_FROMALL, Send_Motor);  
  mySerCmd.AddCmd("GETML", SERIALCMD_FROMALL, Get_ML);
  
  mySerCmd.AddCmd("GETMAP", SERIALCMD_FROMALL, Get_Map);

  // Head Commands
  mySerCmd.AddCmd("H_IDLE", SERIALCMD_FROMALL, set_IDLE);
  mySerCmd.AddCmd("H_LOOK", SERIALCMD_FROMALL, set_LOOK);
  mySerCmd.AddCmd("H_GAZE", SERIALCMD_FROMALL, set_GAZE);

  // Arm Commands
  mySerCmd.AddCmd("A_CAL", SERIALCMD_FROMALL, run_CALIBRATION);
  mySerCmd.AddCmd("A_OPEN", SERIALCMD_FROMALL, set_OPEN);
  mySerCmd.AddCmd("A_CLOSE", SERIALCMD_FROMALL, set_CLOSE);
  mySerCmd.AddCmd("A_ANGLE", SERIALCMD_FROMALL, set_ANGLE);
  mySerCmd.AddCmd("A_ANGLES", SERIALCMD_FROMALL, set_ANGLES);

  // Initialize I2C bus
  Wire.begin();

  // Scan for I2C devices (prevents the application from locking up if an accessory is missing)
  Send_Ping();

  // If attached, configure the time of flight sensors
  if (tofs_attached) {
    tofs_setup();
  }

  // Change the on-board neopixel to green to indicate setup is complete
  onboard_pixel.setPixelColor(0, onboard_pixel.Color(0, 10, 0));
  onboard_pixel.show();

  mySerCmd.Print((char *) "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\r\n");
  mySerCmd.Print((char *) "INFO: Starting application...\r\n");
}


// -------------------------------------------------------
// loop()
// -------------------------------------------------------
void loop() {
  int8_t ret;
  
  // Read ToF sensor values and send a simulated bump command if an object is too close (only run if tof sensors are attached)
  if (tofs_left_state == TOFS_STATE_READY || tofs_right_state == TOFS_STATE_READY) {
    if (check_left_sensor()) {
      mySerCmd.Print((char *) "INFO: Left Object Detected!\r\n");
      //Send_Bump();
      ret = mySerCmd.ReadString((char *) "BUMP,0,1");
    }

    if (check_right_sensor()) {
      mySerCmd.Print((char *) "INFO: Right Object Detected!\r\n");
      //Send_Bump();
      ret = mySerCmd.ReadString((char *) "BUMP,1,0");
    }
  }

  // Check for incoming serial commands
  ret = mySerCmd.ReadSer();
  if (ret == 0) {
    mySerCmd.Print((char *) "ERROR: Urecognized command\r\n");
  }

  // Check for data coming from the SLAM base robot
  if (Serial1.available()) {
    Get_Packet();
  }
}





// Get map command - GETMAP1
// Example - "GETMAP1"
void Get_Map(void) {
  byte get_map1_frame[] = {
    0x55, 0xAA, // HEADER_HI, HEADER_LOW
    0x02, // CTRL_ID
    0x00, 0x06, // LEN_HI, LEN_LOW
    0x21, // PACKET_ID
    0x04, // PACKET_LEN
    0x02, 0x00, 0x00, 0x00, // map_id
    0x73, 0x84 // CRC_HI, CRC_LOW
  };

  byte get_map2_frame[] = {
    0x55, 0xAA, // HEADER_HI, HEADER_LOW
    0x2F, // CTRL_ID_RESP
    0x00, 0x17, // LEN_HI, LEN_LOW
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // no_use0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // no_use1
    0x00, 0x28, 0x00, 0x00, // file_limit = 10240
    0x40, 0x00, // packet_size = 64
    0x00, // no_use2
    0x81, 0x74 // CRC_HI, CRC_LOW
  };

  byte get_map3_frame[] = {
    0x55, 0xAA, // HEADER_HI, HEADER_LOW
    0x30, // CTRL_ID_RESP
    0x00, 0x09, // LEN_HI, LEN_LOW
    0x00, // stat
    0x00, 0x00, 0x00, 0x00, // last_offset
    0x00, 0x00, 0x00, 0x00, // last_crc32
    0x7C, 0xFE // CRC_HI, CRC_LOW
  };

  byte get_map4_frame[] = {
    0x55, 0xAA, // HEADER_HI, HEADER_LOW
    0x31, // CTRL_ID_RESP
    0x00, 0x04, // LEN_HI, LEN_LOW
    0x00, 0x00, 0x00, 0x00, // last_offset
    0x68, 0xEA // CRC_HI, CRC_LOW
  };

  mySerCmd.Print((char *) "INFO: Sending get_map_frame\r\n");
  Send_Frame(get_map1_frame, sizeof(get_map1_frame));
  Get_Packet(get_map1_frame[5]);

  //Get_File_Transfer_Packet(0x0F);

  // Send CTRL_OTA_START_RESP packet - GETMAP2
  mySerCmd.Print((char *) "INFO: Sending CTRL_OTA_START_RESP\r\n");
  Send_Frame(get_map2_frame, sizeof(get_map2_frame));
  Get_File_Transfer_Packet(0x10);
  //Get_Packet();

  // Send CTRL_OTA_FILE_INFO_RESP packet - GETMAP3
  mySerCmd.Print((char *) "INFO: Sending CTRL_OTA_FILE_INFO_RESP\r\n");
  Send_Frame(get_map3_frame, sizeof(get_map3_frame));
  Get_File_Transfer_Packet(0x11);
  //Get_Packet();

  // Send CTRL_OTA_FILE_POS_RESP packet - GETMAP4
  mySerCmd.Print((char *) "INFO: Sending CTRL_OTA_FILE_POS_RESP\r\n");
  Send_Frame(get_map4_frame, sizeof(get_map4_frame));
  Get_File_Transfer_Packet(0x12);
  //Get_Packet();


  sendOK();
}



// -------------------------------------------------------
// General SerialCmd Functions
// -------------------------------------------------------
void sendOK(void) {
  mySerCmd.Print((char *) "OK\r\n");
}

// Sends pings out and listens for responses to see which hardware is attached to the main controller. Then sets the
// approate flags to enable the associated functionality
// Example - "PING"
void Send_Ping(void) {
  mySerCmd.Print((char *)   "INFO: Main Controller           - ATTACHED\r\n");

  Wire.beginTransmission(TEMP_SENSOR_I2C_ADDRESS);
  if (Wire.endTransmission () == 0) {
    temperature_sensor_attached = true;
    mySerCmd.Print((char *) "INFO: Temperature Sensor        - ATTACHED\r\n");
  }

  Wire.beginTransmission(TOFS_DEFAULT_I2C_ADDRESS);
  if (Wire.endTransmission () == 0) {
    tofs_attached = true;
    mySerCmd.Print((char *) "INFO: Time of Flight Sensors    - ATTACHED (not configured)\r\n");
  } else {
    Wire.beginTransmission(TOF_LEFT_I2C_ADDRESS);
    if (Wire.endTransmission () == 0) {
      tofs_attached = true;
      mySerCmd.Print((char *) "INFO: Left ToF Sensor           - ATTACHED\r\n");
    }

    Wire.beginTransmission(TOF_RIGHT_I2C_ADDRESS);
    if (Wire.endTransmission () == 0) {
      tofs_attached = true;
      mySerCmd.Print((char *) "INFO: Right ToF Sensor          - ATTACHED\r\n");
    }
  }

  Wire.beginTransmission(AME_I2C_ADDRESS); // Head Mouth Eyes PCBA
  if (Wire.endTransmission () == 0) {
    head_ame_attached = true;
    mySerCmd.Print((char *) "INFO: Audio/Mouth/Eyes PCBA     - ATTACHED\r\n");
  }

  Wire.beginTransmission(DYN_I2C_ADDRESS); // Dynamixel Contoller PCBA
  if (Wire.endTransmission () == 0) {
    head_dyn_attached = true;
    mySerCmd.Print((char *) "INFO: Head Dynamixel Controller - ATTACHED\r\n");
  }

  Wire.beginTransmission(ARM_I2C_ADDRESS); // Arm Controller PCBA
  if (Wire.endTransmission () == 0) {
    arm_attached = true;
    mySerCmd.Print((char *) "INFO: Arm Controller            - ATTACHED\r\n");
  }

  sendOK();
}


// Reports the versions of the boards connected to the Hackerbot
// Example - "VERSION"
void Get_Version(void) {
  mySerCmd.Print((char *) "INFO: Main Controller (v");
  mySerCmd.Print(VERSION_NUMBER);
  mySerCmd.Print((char *) ".0)\r\n");
  
  if (head_ame_attached) {
    Wire.beginTransmission(AME_I2C_ADDRESS);
    Wire.write(I2C_COMMAND_VERSION);
    Wire.endTransmission();
    Wire.requestFrom(AME_I2C_ADDRESS, 1);
    while(Wire.available()) {
      RxByte = Wire.read();
    }
    mySerCmd.Print((char *) "INFO: Audio Mouth Eyes (v");
    mySerCmd.Print(RxByte);
    mySerCmd.Print((char *) ".0)\r\n");
  }

  if (head_dyn_attached) {
    Wire.beginTransmission(DYN_I2C_ADDRESS);
    Wire.write(I2C_COMMAND_VERSION);
    Wire.endTransmission();
    // I2C RX
    Wire.requestFrom(DYN_I2C_ADDRESS, 1);
    while(Wire.available()) {
      RxByte = Wire.read();
    }
    mySerCmd.Print((char *) "INFO: Dynamixel Controller (v");
    mySerCmd.Print(RxByte);
    mySerCmd.Print((char *) ".0)\r\n");
  }

  if (arm_attached) {
    Wire.beginTransmission(ARM_I2C_ADDRESS);
    Wire.write(0x02);
    Wire.endTransmission();
    // I2C RX
    Wire.requestFrom(ARM_I2C_ADDRESS, 1);
    while(Wire.available()) {
      RxByte = Wire.read();
    }
    mySerCmd.Print((char *) "INFO: Arm Controller (v");
    mySerCmd.Print(RxByte);
    mySerCmd.Print((char *) ".0)\r\n");
  }

  sendOK();
}


// Initializes the connection between the Arduino on the Main Controller with the SLAM vacuum base robot. This must be run once after a power cycle before any other commands can be sent
// Example - "INIT"
void Send_Handshake(void) {
  byte response[88];

  mySerCmd.Print((char *) "INFO: Sending handshake_frame\r\n");
  Send_Frame_Get_Response(handshake_frame, sizeof(handshake_frame), response, sizeof(response));

  response[2] = 0x02;

  mySerCmd.Print((char *) "INFO: Sending handshake acknowledgement frame\r\n");
  Send_Frame(response, sizeof(response));

  Get_Packet(response[5]);
  sendOK();
}


// Set mode control to idle.
// Example - "MODE"
void Send_Mode(void) {
  uint8_t modeParam = 0;

  if (!mySerCmd.ReadNextUInt8(&modeParam)) {
    mySerCmd.Print((char *) "ERROR: Missing parameter\r\n");
    return;
  }

  // Constrain values to acceptable range and set the value into the frame
  modeParam = constrain(modeParam, 0, 12);
  mode_control_frame[7] = modeParam;

  mySerCmd.Print((char *) "INFO: Sending mode_control_frame\r\n");
  Send_Frame(mode_control_frame, sizeof(mode_control_frame));

  Get_Packet(mode_control_frame[5]);
  sendOK();
}


// Gets a list of all of the maps stored on the robot
// Example - "GETML"
void Get_ML(void) {
  mySerCmd.Print((char *) "INFO: Sending get_map_list_frame\r\n");
  Send_Frame(get_map_list_frame, sizeof(get_map_list_frame));

  Get_Packet(get_map_list_frame[5]);
  sendOK();
}


// Set mode to enter. This moves the robot off the dock and starts the LiDAR
// Example - "ENTER"
void Send_Enter(void) {
  behavior_control_frame[7] = 0x00;

  mySerCmd.Print((char *) "INFO: Sending behavior_control_frame (ENTER)\r\n");
  Send_Frame(behavior_control_frame, sizeof(behavior_control_frame));

  Get_Packet(behavior_control_frame[5]);
  sendOK();
}


// Set mode to quick map. This moves the robot off the dock and generates an automatic map of your space
// Example - "QUICKMAP"
void Send_QuickMap(void) {
  behavior_control_frame[7] = 1;

  mySerCmd.Print((char *) "INFO: Sending behavior_control_frame (QUICKMAP)\r\n");
  Send_Frame(behavior_control_frame, sizeof(behavior_control_frame));

  Get_Packet(behavior_control_frame[5]);
  sendOK();
}


// Return to the charger dock and begin charging
// Example - "DOCK"
void Send_Dock(void) {
  behavior_control_frame[7] = 6;

  mySerCmd.Print((char *) "INFO: Sending behavior_control_frame (DOCK)\r\n");
  Send_Frame(behavior_control_frame, sizeof(behavior_control_frame));

  Get_Packet(behavior_control_frame[5]);
  sendOK();
}


// Send a stop command to the base
// Example - "STOP"
void Send_Stop(void) {
  behavior_control_frame[7] = 7;

  mySerCmd.Print((char *) "INFO: Sending behavior_control_frame (STOP)\r\n");
  Send_Frame(behavior_control_frame, sizeof(behavior_control_frame));

  Get_Packet(behavior_control_frame[5]);
  sendOK();
}


// Go to pose command. Must have a map created first.
// Parameters
// float: x (meters), float: y (meters), float: angle (degrees), float: speed (m/s)
// Example - "GOTO,0.5,0.5,0,10"
void Send_Goto(void) {
  float xParam = 0.0;
  float yParam = 0.0;
  float aParam = 0.0;
  float sParam = 0.0;

  if (!mySerCmd.ReadNextFloat(&xParam) || !mySerCmd.ReadNextFloat(&yParam) || !mySerCmd.ReadNextFloat(&aParam) || !mySerCmd.ReadNextFloat(&sParam)) {
    mySerCmd.Print((char *) "ERROR: Missing parameter\r\n");
    return;
  }

  // Constrain values to acceptable range
  xParam = constrain(xParam, -1000.0, 1000.0);
  yParam = constrain(yParam, -1000.0, 1000.0);
  aParam = constrain(aParam, -360.0, 360.0);
  sParam = constrain(sParam, 0.0, 0.4);

  // Convert degrees to radians
  aParam = aParam * (3.1416 / 180.0);

  uint8_t xParamHex[4];
  uint8_t yParamHex[4];
  uint8_t aParamHex[4];
  uint8_t sParamHex[4];

  memcpy(xParamHex, &xParam, sizeof(xParamHex));
  memcpy(yParamHex, &yParam, sizeof(yParamHex));
  memcpy(aParamHex, &aParam, sizeof(aParamHex));
  memcpy(sParamHex, &sParam, sizeof(sParamHex));

  behavior_control_frame[7] = 0x04;

  behavior_control_frame[9]  = xParamHex[0];
  behavior_control_frame[10] = xParamHex[1];
  behavior_control_frame[11] = xParamHex[2];
  behavior_control_frame[12] = xParamHex[3];

  behavior_control_frame[13] = yParamHex[0];
  behavior_control_frame[14] = yParamHex[1];
  behavior_control_frame[15] = yParamHex[2];
  behavior_control_frame[16] = yParamHex[3];

  behavior_control_frame[17] = aParamHex[0];
  behavior_control_frame[18] = aParamHex[1];
  behavior_control_frame[19] = aParamHex[2];
  behavior_control_frame[20] = aParamHex[3];

  behavior_control_frame[21] = sParamHex[0];
  behavior_control_frame[22] = sParamHex[1];
  behavior_control_frame[23] = sParamHex[2];
  behavior_control_frame[24] = sParamHex[3];

  mySerCmd.Print((char *) "INFO: Sending behavior_control_frame (GOTO)\r\n");
  Send_Frame(behavior_control_frame, sizeof(behavior_control_frame));

  Get_Packet(behavior_control_frame[5]);
  sendOK();
}


// Simulated bumper command
// Parameters
// bool: left, bool: right
// Example - "BUMP,1,0"
void Send_Bump(void) {
  uint8_t leftParam = 0;
  uint8_t rightParam = 0;

  if (!mySerCmd.ReadNextUInt8(&leftParam) || !mySerCmd.ReadNextUInt8(&rightParam)) {
    mySerCmd.Print((char *) "ERROR: Missing parameter\r\n");
    return;
  }

  // Constrain values to acceptable range and set the value into the frame
  leftParam = constrain(leftParam, 0, 1);
  rightParam = constrain(rightParam, 0, 1);

  // binary: 0, 0, 0, 0, 0, 0, left, right
  sim_bump_frame[7] = (leftParam << 1) | rightParam;

  mySerCmd.Print((char *) "INFO: Sending sim_bump_frame\r\n");
  Send_Frame(sim_bump_frame, sizeof(sim_bump_frame));

  sendOK();
}


// Wheel motor packet
// Parameters
// int16_t: linear_velocity (mm/s), int16_t: angular_velocity (degrees/s)
// Example - "MOTOR,10,20"
void Send_Motor(void) {
  float linearParam = 0.0;
  float angularParam = 0.0;

  int16_t linearParam16 = 0;
  int16_t angularParam16 = 0;

  if (!mySerCmd.ReadNextFloat(&linearParam) || !mySerCmd.ReadNextFloat(&angularParam)) {
    mySerCmd.Print((char *) "ERROR: Missing parameter\r\n");
    return;
  }

  // Constrain values to acceptable range and set the value into the frame
  linearParam = constrain(linearParam, -100.0, 100.0);
  angularParam = constrain(angularParam, -100.0, 100.0);

  // Convert degrees/s to mrad/s
  angularParam = angularParam * ((1000 * 3.1416) / 180);

  // Convert the floats to signed int16_t
  linearParam16 = (int16_t)linearParam;
  angularParam16 = (int16_t)angularParam;

  wheel_motor_frame[7]  = lowByte(linearParam16);
  wheel_motor_frame[8]  = highByte(linearParam16);

  wheel_motor_frame[9]  = lowByte(angularParam16);
  wheel_motor_frame[10] = highByte(angularParam16);

  mySerCmd.Print((char *) "INFO: Sending wheel_motor_frame\r\n");
  Send_Frame(wheel_motor_frame, sizeof(wheel_motor_frame));

  sendOK();
}


// -------------------------------------------------------
// Head Functions
// -------------------------------------------------------
void set_IDLE(void) {
  char * sParam;
  sParam = mySerCmd.ReadNext();

  if (head_ame_attached == false) {
    mySerCmd.Print((char *) "ERROR: Dynamixel controller not attached\r\n");
    return;
  }
 
  if (sParam == NULL) {
    mySerCmd.Print((char *) "ERROR: Missing idle parameter\r\n");
    return;
  }

  if (strtoul(sParam, NULL, 10) == 0) {
    Wire.beginTransmission(DYN_I2C_ADDRESS);
    Wire.write(I2C_COMMAND_H_IDLE);
    Wire.write(0x00);
    Wire.endTransmission();
    mySerCmd.Print((char *) "INFO: Head idle mode disabled\r\n");
  } else {
    Wire.beginTransmission(DYN_I2C_ADDRESS);
    Wire.write(I2C_COMMAND_H_IDLE);
    Wire.write(0x01);
    Wire.endTransmission();
    mySerCmd.Print((char *) "INFO: Head idle mode enabled\r\n");
  }

  sendOK();
}


void set_LOOK(void) {
  float turnParam = 0.0;
  float vertParam = 0.0;
  uint8_t speedParam = 0;

  if (head_ame_attached == false) {
    mySerCmd.Print((char *) "ERROR: Dynamixel controller not attached\r\n");
    return;
  }
 
  if (!mySerCmd.ReadNextFloat(&turnParam) || !mySerCmd.ReadNextFloat(&vertParam) || !mySerCmd.ReadNextUInt8(&speedParam)) {
    mySerCmd.Print((char *) "ERROR: Missing parameter\r\n");
    return;
  }

  // Constrain values to acceptable range
  turnParam = constrain(turnParam, 100.0, 260.0);
  vertParam = constrain(vertParam, 150.0, 250.0);
  speedParam = constrain(speedParam, 6, 70);

  char buf[128] = {0};
  sprintf(buf, "INFO: Looking to position turn: %0.2f, vert: %0.2f, at speed: %d\r\n", turnParam, vertParam, speedParam);
  mySerCmd.Print(buf);

  uint16_t turnParam16 = (uint16_t)(turnParam * 10);
  uint16_t vertParam16 = (uint16_t)(vertParam * 10);
  uint8_t speedParam8 = (uint8_t)(speedParam);

  Wire.beginTransmission(DYN_I2C_ADDRESS);
  Wire.write(I2C_COMMAND_H_LOOK);
  Wire.write(highByte(turnParam16)); // Yaw H
  Wire.write(lowByte(turnParam16)); // YAW L
  Wire.write(highByte(vertParam16)); // Pitch H
  Wire.write(lowByte(vertParam16)); // Pitch L
  Wire.write(speedParam8);
  Wire.endTransmission();

  sendOK();
}


void set_GAZE(void) {
  float eyeTargetX = 0.0;
  float eyeTargetY = 0.0;

  if (head_ame_attached == false) {
    mySerCmd.Print((char *) "ERROR: Audio/Mouth/Eyes controller not attached\r\n");
    return;
  }

  if (!mySerCmd.ReadNextFloat(&eyeTargetX) || !mySerCmd.ReadNextFloat(&eyeTargetY)) {
    mySerCmd.Print((char *) "ERROR: Missing parameter\r\n");
    return;
  }

  // Constrain values to acceptable range
  eyeTargetX = constrain(eyeTargetX, -1.0, 1.0);
  eyeTargetY = constrain(eyeTargetY, -1.0, 1.0);

  char buf[128] = {0};
  sprintf(buf, "INFO: Setting: eyeTargetX: %0.2f, eyeTargetY: %0.2f\r\n", eyeTargetX, eyeTargetY);
  mySerCmd.Print(buf);

  // scale to fit an int8 for smaller i2c transport
  int8_t eyeTargetXInt8 = int8_t(eyeTargetX * 100.0);
  int8_t eyeTargetYInt8 = int8_t(eyeTargetY * 100.0);

  Wire.beginTransmission(AME_I2C_ADDRESS);
  Wire.write(I2C_COMMAND_H_GAZE);
  Wire.write((uint8_t)eyeTargetXInt8);
  Wire.write((uint8_t)eyeTargetYInt8);
  Wire.endTransmission();

  sendOK();
}


// -------------------------------------------------------
// Arm Functions
// -------------------------------------------------------
// A_CAL
void run_CALIBRATION(void) {
  mySerCmd.Print((char *) "INFO: Calibrating the gripper\r\n");

  Wire.beginTransmission(ARM_I2C_ADDRESS);
  Wire.write(I2C_COMMAND_ARM_CALIBRATION);
  Wire.endTransmission();

  sendOK();
}

// A_OPEN
void set_OPEN(void) {
  mySerCmd.Print((char *) "INFO: Opening the gripper\r\n");

  Wire.beginTransmission(ARM_I2C_ADDRESS);
  Wire.write(I2C_COMMAND_ARM_OPEN);
  Wire.endTransmission();
  
  sendOK();
}

// A_CLOSE
void set_CLOSE(void) {
  mySerCmd.Print((char *) "INFO: Closing the gripper\r\n");

  Wire.beginTransmission(ARM_I2C_ADDRESS);
  Wire.write(I2C_COMMAND_ARM_CLOSE);
  Wire.endTransmission();
  
  sendOK();
}

// A_ANGLE
void set_ANGLE(void) {
  uint8_t jointParam = 0;
  float angleParam = 0.0;
  uint8_t speedParam = 0;

  if (!mySerCmd.ReadNextUInt8(&jointParam) ||
      !mySerCmd.ReadNextFloat(&angleParam) ||
      !mySerCmd.ReadNextUInt8(&speedParam)) {
    mySerCmd.Print((char *) "ERROR: Missing parameter\r\n");
    return;
  }

  // Constrain values to acceptable range
  jointParam = constrain(jointParam, 1, 6);
  float limit = LIMIT_FOR_ARM_JOINT(jointParam);
  angleParam = constrain(angleParam, -1 * limit, limit);
  speedParam = constrain(speedParam, 0, 100);

  char buf[128] = {0};
  sprintf(buf, "INFO: Setting the angle of joint %d to %0.1f degrees at speed %d\r\n", jointParam, angleParam, speedParam);
  mySerCmd.Print(buf);

  uint16_t angleParam16 = hb_ftoi(angleParam);

  Wire.beginTransmission(ARM_I2C_ADDRESS);
  Wire.write(I2C_COMMAND_ARM_ANGLE);
  Wire.write(jointParam);
  Wire.write(highByte(angleParam16)); // Angle H
  Wire.write(lowByte(angleParam16)); // Angle L
  Wire.write(speedParam);
  Wire.endTransmission();

  sendOK();
}

// A_ANGLES
void set_ANGLES(void) {
  float jointParam[6] = {0.0};
  uint8_t speedParam = 0;

  if (!mySerCmd.ReadNextFloat(&jointParam[0]) ||
      !mySerCmd.ReadNextFloat(&jointParam[1]) ||
      !mySerCmd.ReadNextFloat(&jointParam[2]) ||
      !mySerCmd.ReadNextFloat(&jointParam[3]) ||
      !mySerCmd.ReadNextFloat(&jointParam[4]) ||
      !mySerCmd.ReadNextFloat(&jointParam[5]) ||
      !mySerCmd.ReadNextUInt8(&speedParam)) {
    mySerCmd.Print((char *) "ERROR: Missing parameter\r\n");
    return;
  }

  // Constrain values to acceptable range
  for (int ndx = 0; ndx < 6; ndx++) {
    float limit = LIMIT_FOR_ARM_JOINT(ndx+1);
    jointParam[ndx] = constrain(jointParam[ndx], -1 * limit, limit);
  }
  speedParam = constrain(speedParam, 0, 100);

  char buf[160] = {0};
  char *pos = buf;
  pos += sprintf(pos, "INFO: Setting the angle of the joints to");
  for (int ndx = 0; ndx < 6; ndx++) {
    pos += sprintf(pos, " (%d) %.1f%s", ndx + 1, jointParam[ndx], (ndx < 5 ? "," : ""));
  }
  sprintf(pos, " degrees at speed %d\r\n", speedParam);
  mySerCmd.Print(buf);

  Wire.beginTransmission(ARM_I2C_ADDRESS);
  Wire.write(I2C_COMMAND_ARM_ANGLES);
  for (int ndx = 0; ndx < 6; ndx++) {
    uint16_t jointParam16 = hb_ftoi(jointParam[ndx]);
    Wire.write(highByte(jointParam16)); // Joint Angle H
    Wire.write(lowByte(jointParam16)); // Joint Angle L
  }
  Wire.write(speedParam);
  Wire.endTransmission();

  sendOK();
}


// -------------------------------------------------------
// General Helper Functions
// -------------------------------------------------------
void Get_Packet(byte response_packetid, byte response_frame[], int sizeOfResponseFrame) {
  unsigned long responseTimeout = millis();
  const int response_timeout_period = 5000;

  const int incomingPacketBuffSize = 320;
  uint8_t incomingByte;
  uint8_t incomingPacket[incomingPacketBuffSize] = {0};
  int incomingPacketLen = 0;
  uint8_t lenByteHigh;
  uint8_t lenByteLow;
  uint16_t lenByte;
  uint8_t packetID;

  bool responseByteReceived = false;

  char hexString[3];
  const char hexChars[] = "0123456789ABCDEF";

  while (!responseByteReceived) {
    // If the Get_Packet function is called but the user doesn't care what PACKETID the response has, then only read
    // and display one packet. Otherwise, keep reading packets until the one with the requested PACKET_ID comes in
    if (response_packetid == 0x00) {
      responseByteReceived = true;
    }

    while (incomingPacket[0] != 0x55 && incomingPacket[1] != 0xAA) {
      while (!Serial1.available()) {
        if (millis() - responseTimeout >= response_timeout_period) {
            mySerCmd.Print((char *) "WARNING: Timed out while waiting for the first header byte in the response packet (0x55)\r\n");
            return;
        }
      }

      incomingByte = Serial1.read();
      incomingPacket[incomingPacketLen] = incomingByte;

      if (incomingPacket[incomingPacketLen] != 0x55) {
        if (responseByteReceived) {
          mySerCmd.Print((char *) "WARNING: Unexpected byte received (not 0x55) ");
          hexString[0] = hexChars[incomingPacket[incomingPacketLen] >> 4];
          hexString[1] = hexChars[incomingPacket[incomingPacketLen] & 0x0F];
          hexString[2] = '\0';
          mySerCmd.Print(hexString);
          mySerCmd.Print((char *) "\r\n");
        }
      } else {
        incomingPacketLen++;
        
        while (!Serial1.available()) {
          if (millis() - responseTimeout >= response_timeout_period) {
            mySerCmd.Print((char *) "WARNING: Timed out while waiting for the second header byte in the response packet (0xAA)\r\n");
            return;
          }
        }

        incomingByte = Serial1.read();
        incomingPacket[incomingPacketLen] = incomingByte;

        if(incomingPacket[incomingPacketLen] != 0xAA) {
          if (responseByteReceived) {
            mySerCmd.Print((char *) "WARNING: Unexpected byte received (not 0xAA) ");
            hexString[0] = hexChars[incomingPacket[incomingPacketLen] >> 4];
            hexString[1] = hexChars[incomingPacket[incomingPacketLen] & 0x0F];
            hexString[2] = '\0';
            mySerCmd.Print(hexString);
            mySerCmd.Print((char *) "\r\n");
          }

          incomingPacketLen = 0;
        }
      }
    }

    incomingPacketLen++;

    // Wait for and receive the CTRL_ID
    while(!Serial1.available()) {
      if(millis() - responseTimeout >= response_timeout_period) {
          mySerCmd.Print((char *) "WARNING: Sending frame timed out while waiting for the CTRL_ID in the response packet\r\n");
          return;
      }
    }

    incomingByte = Serial1.read();
    incomingPacket[incomingPacketLen] = incomingByte;
    incomingPacketLen++;

    // Wait for and receive the LEN_HI
    while(!Serial1.available()) {
      if(millis() - responseTimeout >= response_timeout_period) {
          mySerCmd.Print((char *) "WARNING: Sending frame timed out while waiting for the LEN_HI in the response packet\r\n");
          return;
      }
    }

    incomingByte = Serial1.read();
    incomingPacket[incomingPacketLen] = incomingByte;
    lenByteHigh = incomingPacket[incomingPacketLen];
    incomingPacketLen++;

    // Wait for and receive the LEN_LOW
    while(!Serial1.available()) {
      if(millis() - responseTimeout >= response_timeout_period) {
          mySerCmd.Print((char *) "WARNING: Sending frame timed out while waiting for the LEN_LOW in the response packet\r\n");
          return;
      }
    }

    incomingByte = Serial1.read();
    incomingPacket[incomingPacketLen] = incomingByte;
    lenByteLow = incomingPacket[incomingPacketLen];
    incomingPacketLen++;

    lenByte = (lenByteHigh << 8) | lenByteLow;

    // Wait for and receive the PACKET_ID
    while(!Serial1.available()) {
      if(millis() - responseTimeout >= response_timeout_period) {
          mySerCmd.Print((char *) "WARNING: Sending frame timed out while waiting for the PACKET_ID in the response packet\r\n");
          return;
      }
    }

    incomingByte = Serial1.read();
    incomingPacket[incomingPacketLen] = incomingByte;
    packetID = incomingPacket[incomingPacketLen];
    incomingPacketLen++;

    // Wait for and receive the remaining packet data
    for (int i = 0; i < (lenByte + 1); i++) {
      while(!Serial1.available()) {
        if(millis() - responseTimeout >= response_timeout_period) {
            mySerCmd.Print((char *) "WARNING: Sending frame timed out while waiting for the DATA or CRC in the response packet\r\n");
            return;
        }
      }

      if (lenByte <= (incomingPacketBuffSize - 7)) {
        incomingByte = Serial1.read();
        incomingPacket[incomingPacketLen] = incomingByte;
        incomingPacketLen++;
      } else {
        incomingByte = Serial1.read();
      }
    }

    if (lenByte > (incomingPacketBuffSize - 7)) {
      mySerCmd.Print((char *) "WARNING: Packet length is too long for the incomingPacket buffer. Throwing out ");
      mySerCmd.Print(lenByte + 7);
      mySerCmd.Print((char *) " bytes from PACKET_ID 0x");
      hexString[0] = hexChars[packetID >> 4];
      hexString[1] = hexChars[packetID & 0x0F];
      hexString[2] = '\0';
      mySerCmd.Print(hexString);
      mySerCmd.Print((char *) "\r\n");
      return;
    }

    // Don't display...
    // Status Packets = 0x02
    // Pose Packets = 0x23
    if ((incomingPacket[5] == 0x02 || incomingPacket[5] == 0x23) && responseByteReceived) {
      return;
    }

    // If we're looking for a specific response packet, check to see if we got that packet 
    if (response_packetid != 0x00) {
      if (incomingPacket[5] == response_packetid) {
        responseByteReceived = true;
      } else {
        incomingPacket[0] = 0x00;
        incomingPacket[1] = 0x00;
        incomingPacketLen = 0;
      }
    }
  }

  mySerCmd.Print((char *) "INFO: Received    ");
  for (int i = 0; i < incomingPacketLen; i++) {
    if (response_frame != nullptr) {
      if (i < sizeOfResponseFrame) {
        response_frame[i] = incomingPacket[i];
      }
    }
    hexString[0] = hexChars[incomingPacket[i] >> 4];
    hexString[1] = hexChars[incomingPacket[i] & 0x0F];
    hexString[2] = '\0';
    mySerCmd.Print(hexString);
    mySerCmd.Print((char *) " ");
  }
  mySerCmd.Print((char *) " (");
  mySerCmd.Print(incomingPacketLen);
  mySerCmd.Print((char *) ")\r\n");

  if (response_frame != nullptr) {
    if (incomingPacketLen != sizeOfResponseFrame) {
      mySerCmd.Print((char *) "WARNING: Length of the frame received does not match the length that was specified\r\n");
    }
  }
}


// Get the request packet for a file transfer
void Get_File_Transfer_Packet(byte request_ctrlid) {
  unsigned long responseTimeout = millis();
  const int response_timeout_period = 5000;

  const int incomingPacketBuffSize = 320;
  uint8_t incomingByte;
  uint8_t incomingPacket[incomingPacketBuffSize] = {0};
  int incomingPacketLen = 0;
  uint8_t lenByteHigh;
  uint8_t lenByteLow;
  uint16_t lenByte;
  uint8_t ctrlID;

  bool requestByteReceived = false;

  char hexString[3];
  const char hexChars[] = "0123456789ABCDEF";

  while (!requestByteReceived) {
    // If the Get_Packet function is called but the user doesn't care what PACKETID the response has, then only read
    // and display one packet. Otherwise, keep reading packets until the one with the requested PACKET_ID comes in
    if (request_ctrlid == 0x00) {
      requestByteReceived = true;
    }

    while (incomingPacket[0] != 0x55 && incomingPacket[1] != 0xAA) {
      while (!Serial1.available()) {
        if (millis() - responseTimeout >= response_timeout_period) {
            mySerCmd.Print((char *) "WARNING: Timed out while waiting for the first header byte in the response packet (0x55)\r\n");
            return;
        }
      }

      incomingByte = Serial1.read();
      incomingPacket[incomingPacketLen] = incomingByte;

      if (incomingPacket[incomingPacketLen] != 0x55) {
        if (requestByteReceived) {
          mySerCmd.Print((char *) "WARNING: Unexpected byte received (not 0x55) ");
          hexString[0] = hexChars[incomingPacket[incomingPacketLen] >> 4];
          hexString[1] = hexChars[incomingPacket[incomingPacketLen] & 0x0F];
          hexString[2] = '\0';
          mySerCmd.Print(hexString);
          mySerCmd.Print((char *) "\r\n");
        }
      } else {
        incomingPacketLen++;
        
        while (!Serial1.available()) {
          if (millis() - responseTimeout >= response_timeout_period) {
            mySerCmd.Print((char *) "WARNING: Timed out while waiting for the second header byte in the response packet (0xAA)\r\n");
            return;
          }
        }

        incomingByte = Serial1.read();
        incomingPacket[incomingPacketLen] = incomingByte;

        if(incomingPacket[incomingPacketLen] != 0xAA) {
          if (requestByteReceived) {
            mySerCmd.Print((char *) "WARNING: Unexpected byte received (not 0xAA) ");
            hexString[0] = hexChars[incomingPacket[incomingPacketLen] >> 4];
            hexString[1] = hexChars[incomingPacket[incomingPacketLen] & 0x0F];
            hexString[2] = '\0';
            mySerCmd.Print(hexString);
            mySerCmd.Print((char *) "\r\n");
          }

          incomingPacketLen = 0;
        }
      }
    }

    incomingPacketLen++;

    // Wait for and receive the CTRL_ID
    while(!Serial1.available()) {
      if(millis() - responseTimeout >= response_timeout_period) {
          mySerCmd.Print((char *) "WARNING: Sending frame timed out while waiting for the CTRL_ID in the response packet\r\n");
          return;
      }
    }

    incomingByte = Serial1.read();
    incomingPacket[incomingPacketLen] = incomingByte;
    ctrlID = incomingPacket[incomingPacketLen];
    incomingPacketLen++;

    // Wait for and receive the LEN_HI
    while(!Serial1.available()) {
      if(millis() - responseTimeout >= response_timeout_period) {
          mySerCmd.Print((char *) "WARNING: Sending frame timed out while waiting for the LEN_HI in the response packet\r\n");
          return;
      }
    }

    incomingByte = Serial1.read();
    incomingPacket[incomingPacketLen] = incomingByte;
    lenByteHigh = incomingPacket[incomingPacketLen];
    incomingPacketLen++;

    // Wait for and receive the LEN_LOW
    while(!Serial1.available()) {
      if(millis() - responseTimeout >= response_timeout_period) {
          mySerCmd.Print((char *) "WARNING: Sending frame timed out while waiting for the LEN_LOW in the response packet\r\n");
          return;
      }
    }

    incomingByte = Serial1.read();
    incomingPacket[incomingPacketLen] = incomingByte;
    lenByteLow = incomingPacket[incomingPacketLen];
    incomingPacketLen++;

    lenByte = (lenByteHigh << 8) | lenByteLow;

    byte offset = 0;

    if (ctrlID == 0x03) {
      offset = 2;
    }

    // File transfer DATA packet with CRC32
    if (ctrlID == 0x12) {
      offset = 4;
    }

    // Wait for and receive the remaining packet data
    for (int i = 0; i < (lenByte + offset); i++) {
      while(!Serial1.available()) {
        if(millis() - responseTimeout >= response_timeout_period) {
            mySerCmd.Print((char *) "WARNING: Sending frame timed out while waiting for the DATA or CRC in the response packet\r\n");
            return;
        }
      }

      if (lenByte <= (incomingPacketBuffSize - 5)) {
        incomingByte = Serial1.read();
        incomingPacket[incomingPacketLen] = incomingByte;
        incomingPacketLen++;
      } else {
        incomingByte = Serial1.read();
      }
    }

    if (lenByte > (incomingPacketBuffSize - 5)) {
      mySerCmd.Print((char *) "WARNING: Packet length is too long for the incomingPacket buffer. Throwing out ");
      mySerCmd.Print(lenByte + 7);
      mySerCmd.Print((char *) " bytes from CTRL_ID 0x");
      hexString[0] = hexChars[ctrlID >> 4];
      hexString[1] = hexChars[ctrlID & 0x0F];
      hexString[2] = '\0';
      mySerCmd.Print(hexString);
      mySerCmd.Print((char *) "\r\n");
      return;
    }

    if (incomingPacket[2] == request_ctrlid) {
      requestByteReceived = true;
    } else {
      incomingPacket[0] = 0x00;
      incomingPacket[1] = 0x00;
      incomingPacketLen = 0;
    }
  }

  mySerCmd.Print((char *) "INFO: FP Received ");
  for (int i = 0; i < incomingPacketLen; i++) {
    //if (response_frame != nullptr) {
    //  if (i < sizeOfResponseFrame) {
    //    response_frame[i] = incomingPacket[i];
    //  }
    //}
    hexString[0] = hexChars[incomingPacket[i] >> 4];
    hexString[1] = hexChars[incomingPacket[i] & 0x0F];
    hexString[2] = '\0';
    mySerCmd.Print(hexString);
    mySerCmd.Print((char *) " ");
  }
  mySerCmd.Print((char *) " (");
  mySerCmd.Print(incomingPacketLen);
  mySerCmd.Print((char *) ")\r\n");

  //if (response_frame != nullptr) {
  //if (incomingPacketLen != sizeOfResponseFrame) {
  //  mySerCmd.Print((char *) "WARNING: Length of the frame received does not match the length that was specified\r\n");
  //}
  //}
}


// Sends a frame
void Send_Frame(byte frame[], int sizeOfFrame) {
  uint8_t crcHex[2];
  uint16_t crc = crc16_compute(&frame[2], sizeOfFrame - 4);

  char hexString[3];
  const char hexChars[] = "0123456789ABCDEF";

  memcpy(crcHex, &crc, sizeof(crcHex));
  frame[sizeOfFrame - 2] = crcHex[1];
  frame[sizeOfFrame - 1] = crcHex[0];

  mySerCmd.Print((char *) "INFO: Transmitted ");
  for(int i = 0; i < sizeOfFrame; i++) {
    hexString[0] = hexChars[frame[i] >> 4];
    hexString[1] = hexChars[frame[i] & 0x0F];
    hexString[2] = '\0';
    mySerCmd.Print(hexString);
    mySerCmd.Print((char *) " ");
  }
  mySerCmd.Print((char *) "\r\n");

  Serial1.write(frame, sizeOfFrame);
}

// Sends a frame and then saves the response frame with the matching packet id
void Send_Frame_Get_Response(byte frame[], int sizeOfFrame, byte response_frame[], int sizeOfResponseFrame) {
  Send_Frame(frame, sizeOfFrame);
  Get_Packet(frame[5], response_frame, sizeOfResponseFrame);
}


// -------------------------------------------------------
// CRC-16 Functions
// CRC-16/XMODEM https://crccalc.com
// -------------------------------------------------------
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


static uint16_t crc16_compute(byte *data, uint16_t length) {
  uint16_t crc = 0;
  while (length--) {
      crc = (crc << 8) ^ crc16_table[((crc >> 8) ^ *data++) & 0x00FF];
  }

  return crc;
}

// -------------------------------------------------------
// CRC-32 Function
// -------------------------------------------------------
uint32_t crc32_compute(uint8_t const *p_data, uint32_t size, uint32_t const *p_crc)
{
    uint32_t crc;
    crc = (p_crc == NULL) ? 0xFFFFFFFF : ~(*p_crc);
    for (uint32_t i = 0; i < size; i++)
    {
        crc = crc ^ p_data[i];
        for (uint32_t j = 8; j > 0; j--)
        {
            crc = (crc >> 1) ^ (0xEDB88320U & ((crc & 1) ? 0xFFFFFFFF : 0));
        }
    }
    return ~crc;
}