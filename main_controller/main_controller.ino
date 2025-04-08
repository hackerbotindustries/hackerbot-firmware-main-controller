/*********************************************************************************
Hackerbot Industries, LLC
Created By: Ian Bernstein
Created:    April 2024
Updated:    April 1, 2025

This sketch is written for the "Main Controller" PCBA. It serves several funtions:
  1) Communicate with the SLAM Base Robot
  2) Communicate (if connected) with the Head and Arm
  3) Handle the front time of flight sensors for object detection and avoidance
  4) Get temperature data from the on-board temperature sensor
  5) Handle user code for any user added I2C devices (eg QWIIC or STEMMA sensors)

Special thanks to the following for their code contributions to this codebase:
Ian Bernstein - https://github.com/arobodude
Skylar Castator - https://github.com/SkylarCastator
Randy Beiter - https://github.com/rbeiter
*********************************************************************************/


#include <Wire.h>
#include <SerialCmd.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include <vl53l7cx_class.h>

#include "Hackerbot_Shared.h"
#include "SerialCmd_Helper.h"
#include "ToFs_Helper.h"
#include "SLAM_Base_Frames.h"

// Main Controller software version
#define VERSION_NUMBER 10

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

bool tofs_active = true;
bool json_mode = false;

unsigned long previousTofMillis = millis();
bool read_left_tof_toggle = true;
bool left_tof_obj_detected = false;
bool right_tof_obj_detected = false;

// Function prototypes
void Get_Packet(byte response_packetid = 0x00, byte response_frame[] = nullptr, int sizeOfResponseFrame = 0);
void Get_File_Transfer_Packet(byte request_ctrlid = 0x00, uint32_t* currentCRC32 = nullptr, uint32_t* lastCRC32 = nullptr, uint8_t* doneFlag = nullptr);
void Generate_Json_Mode(byte response_frame[] = nullptr, int sizeOfResponseFrame = 0);


// -------------------------------------------------------
// setup()
// -------------------------------------------------------
void setup() {
  unsigned long serialTimout = millis();

  Serial.begin(230400);
  while(!Serial && millis() - serialTimout <= 5000);

  Serial1.begin(230400);

  delay(1000);

  if (!json_mode) mySerCmd.Print((char *) "INFO: Initalizing application...\r\n");
  if (!json_mode) mySerCmd.Print((char *) "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\r\n");

  onboard_pixel.begin();
  onboard_pixel.setPixelColor(0, onboard_pixel.Color(0, 0, 10));
  onboard_pixel.show();

  // Command Setup
  mySerCmd.AddCmd("PING", SERIALCMD_FROMALL, Send_Ping);
  mySerCmd.AddCmd("VERSION", SERIALCMD_FROMALL, Get_Version);
  mySerCmd.AddCmd("JSON", SERIALCMD_FROMALL, Set_Json);
  mySerCmd.AddCmd("TOFS", SERIALCMD_FROMALL, Set_Tofs);
  mySerCmd.AddCmd("B_INIT", SERIALCMD_FROMALL, Send_Handshake);
  mySerCmd.AddCmd("B_MODE", SERIALCMD_FROMALL, Send_Mode);
  mySerCmd.AddCmd("B_START", SERIALCMD_FROMALL, Send_Start);
  mySerCmd.AddCmd("B_QUICKMAP", SERIALCMD_FROMALL, Send_QuickMap);
  mySerCmd.AddCmd("B_GOTO", SERIALCMD_FROMALL, Send_Goto);
  mySerCmd.AddCmd("B_DOCK", SERIALCMD_FROMALL, Send_Dock);
  mySerCmd.AddCmd("B_KILL", SERIALCMD_FROMALL, Send_Kill);
  mySerCmd.AddCmd("B_BUMP", SERIALCMD_FROMALL, Send_Bump);
  mySerCmd.AddCmd("B_DRIVE", SERIALCMD_FROMALL, Send_Drive);  
  mySerCmd.AddCmd("B_MAPLIST", SERIALCMD_FROMALL, Get_Maplist);
  mySerCmd.AddCmd("B_MAPDATA", SERIALCMD_FROMALL, Get_Mapdata);
  mySerCmd.AddCmd("B_STATUS", SERIALCMD_FROMALL, Get_Status);
  mySerCmd.AddCmd("B_POSE", SERIALCMD_FROMALL, Get_Pose);

  // Head Commands
  mySerCmd.AddCmd("H_IDLE", SERIALCMD_FROMALL, set_H_IDLE);
  mySerCmd.AddCmd("H_LOOK", SERIALCMD_FROMALL, set_H_LOOK);
  mySerCmd.AddCmd("H_GAZE", SERIALCMD_FROMALL, set_H_GAZE);

  // Arm Commands
  mySerCmd.AddCmd("A_CAL", SERIALCMD_FROMALL, run_A_CAL);
  mySerCmd.AddCmd("A_OPEN", SERIALCMD_FROMALL, set_A_OPEN);
  mySerCmd.AddCmd("A_CLOSE", SERIALCMD_FROMALL, set_A_CLOSE);
  mySerCmd.AddCmd("A_ANGLE", SERIALCMD_FROMALL, set_A_ANGLE);
  mySerCmd.AddCmd("A_ANGLES", SERIALCMD_FROMALL, set_A_ANGLES);

  // Initialize I2C bus
  Wire.begin();

  // Scan for I2C devices (prevents the application from locking up if an accessory is missing)
  Send_Ping();

  // If attached, configure the time of flight sensors
  if (tofs_attached) {
    tofs_setup();
  }

  // Flush the Serial1 rx buffer
  while (Serial1.available()) {
    Serial1.read();
  }

  // Change the on-board neopixel to green to indicate setup is complete
  onboard_pixel.setPixelColor(0, onboard_pixel.Color(0, 10, 0));
  onboard_pixel.show();

  if (!json_mode) mySerCmd.Print((char *) "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\r\n");
  if (!json_mode) mySerCmd.Print((char *) "INFO: Starting application...\r\n");
}


// -------------------------------------------------------
// loop()
// -------------------------------------------------------
void loop() {
  int8_t ret;
  
  // Read ToF sensor values and send a simulated bump command if an object is too close (only run if the
  // tof sensors are attached, ready, and >250 milliseconds have passed since the last BUMP command was sent).
  // Reading a ToF takes ~50ms each so the reading is split up to help avoid the Serial1 RX buffer from
  // overflowing. Status frames from the SLAM base robot come in every 10ms so it can fill up quickly.
  if (tofs_attached && tofs_active) {
    if (tof_left_state == TOF_STATE_READY && tof_right_state == TOF_STATE_READY) {
      unsigned long currentTofMillis = millis();

      if (currentTofMillis - previousTofMillis >= 125 && read_left_tof_toggle) {
        left_tof_obj_detected = check_left_sensor();
        read_left_tof_toggle = false;
      }

      if (currentTofMillis - previousTofMillis >= 250) {
        previousTofMillis = currentTofMillis;
        read_left_tof_toggle = true;

        right_tof_obj_detected = check_right_sensor();

        if (left_tof_obj_detected && right_tof_obj_detected) {
          if (!json_mode) mySerCmd.Print((char *) "INFO: Both ToFs detect an obstacle\r\n");
          ret = mySerCmd.ReadString((char *) "BUMP,1,1");
        } else if (left_tof_obj_detected) {
          if (!json_mode) mySerCmd.Print((char *) "INFO: Left ToF detects an obstacle\r\n");
          ret = mySerCmd.ReadString((char *) "BUMP,0,1");
        } else if (right_tof_obj_detected) {
          if (!json_mode) mySerCmd.Print((char *) "INFO: Right ToF detects an obstacle\r\n");
          ret = mySerCmd.ReadString((char *) "BUMP,1,0");
        }
      }
    }
  }

  // Check for incoming serial commands
  ret = mySerCmd.ReadSer();
  if (ret == 0) {
    if (!json_mode) mySerCmd.Print((char *) "ERROR: Unrecognized command\r\n");
  }

  // Check for data coming from the SLAM base robot
  if (Serial1.available()) {
  //unsigned int bytesInBuffer = Serial1.available();
  //if (bytesInBuffer) {
  //  if (!json_mode) mySerCmd.Print((char *) "DEBUG: Buffer ");
  //  if (!json_mode) mySerCmd.Print(bytesInBuffer);
  //  if (!json_mode) mySerCmd.Print((char *) "\r\n");
    Get_Packet();
  }
}


// -------------------------------------------------------
// General SerialCmd Functions
// -------------------------------------------------------
void sendOK(void) {
  if (!json_mode) mySerCmd.Print((char *) "OK\r\n");
}


// Sends pings out and listens for responses to see which hardware is attached to the main controller. Then sets the
// approate flags to enable the associated functionality
// Example - "PING"
void Send_Ping(void) {
  JsonDocument json;

  if (json_mode) json["success"] = "true";
  if (json_mode) json["command"] = "ping";

  if (!json_mode) mySerCmd.Print((char *)   "INFO: Main Controller           - ATTACHED\r\n");
  if (json_mode) json["main_controller"] = "attached";

  Wire.beginTransmission(TEMP_SENSOR_I2C_ADDRESS);
  if (Wire.endTransmission () == 0) {
    temperature_sensor_attached = true;
    if (!json_mode) mySerCmd.Print((char *) "INFO: Temperature Sensor        - ATTACHED\r\n");
    if (json_mode) json["temperature_sensor"] = "attached";
  }

  Wire.beginTransmission(TOFS_DEFAULT_I2C_ADDRESS);
  if (Wire.endTransmission () == 0) {
    tofs_attached = true;
    if (!json_mode) mySerCmd.Print((char *) "INFO: Time of Flight Sensors    - ATTACHED (not configured)\r\n");
    if (json_mode) json["tofs"] = "attached";
  } else {
    Wire.beginTransmission(TOF_LEFT_I2C_ADDRESS);
    if (Wire.endTransmission () == 0) {
      tofs_attached = true;
      if (!json_mode) mySerCmd.Print((char *) "INFO: Left ToF Sensor           - ATTACHED\r\n");
      if (json_mode) json["left_tof"] = "attached";
    }

    Wire.beginTransmission(TOF_RIGHT_I2C_ADDRESS);
    if (Wire.endTransmission () == 0) {
      tofs_attached = true;
      if (!json_mode) mySerCmd.Print((char *) "INFO: Right ToF Sensor          - ATTACHED\r\n");
      if (json_mode) json["right_tof"] = "attached";
    }
  }

  Wire.beginTransmission(AME_I2C_ADDRESS); // Head Mouth Eyes PCBA
  if (Wire.endTransmission () == 0) {
    head_ame_attached = true;
    if (!json_mode) mySerCmd.Print((char *) "INFO: Audio/Mouth/Eyes PCBA     - ATTACHED\r\n");
    if (json_mode) json["audio_mouth_eyes"] = "attached";
  }

  Wire.beginTransmission(DYN_I2C_ADDRESS); // Dynamixel Contoller PCBA
  if (Wire.endTransmission () == 0) {
    head_dyn_attached = true;
    if (!json_mode) mySerCmd.Print((char *) "INFO: Head Dynamixel Controller - ATTACHED\r\n");
    if (json_mode) json["dynamixel_controller"] = "attached";
  }

  Wire.beginTransmission(ARM_I2C_ADDRESS); // Arm Controller PCBA
  if (Wire.endTransmission () == 0) {
    arm_attached = true;
    if (!json_mode) mySerCmd.Print((char *) "INFO: Arm Controller            - ATTACHED\r\n");
    if (json_mode) json["arm_controller"] = "attached";
  }

  if (json_mode) { serializeJson(json, Serial); Serial.println(); }
  sendOK();
}


// Reports the versions of the boards connected to the Hackerbot
// Example - "VERSION"
void Get_Version(void) {
  JsonDocument json;

  if (json_mode) json["success"] = "true";
  if (json_mode) json["command"] = "version";

  if (!json_mode) mySerCmd.Print((char *) "INFO: Main Controller (v");
  if (!json_mode) mySerCmd.Print(VERSION_NUMBER);
  if (!json_mode) mySerCmd.Print((char *) ".0)\r\n");
  if (json_mode) json["main_controller"] = VERSION_NUMBER;
  
  if (head_ame_attached) {
    Wire.beginTransmission(AME_I2C_ADDRESS);
    Wire.write(I2C_COMMAND_VERSION);
    Wire.endTransmission();
    Wire.requestFrom(AME_I2C_ADDRESS, 1);
    while(Wire.available()) {
      RxByte = Wire.read();
    }
    if (!json_mode) mySerCmd.Print((char *) "INFO: Audio Mouth Eyes (v");
    if (!json_mode) mySerCmd.Print(RxByte);
    if (!json_mode) mySerCmd.Print((char *) ".0)\r\n");
    if (json_mode) json["audio_mouth_eyes"] = RxByte;
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
    if (!json_mode) mySerCmd.Print((char *) "INFO: Dynamixel Controller (v");
    if (!json_mode) mySerCmd.Print(RxByte);
    if (!json_mode) mySerCmd.Print((char *) ".0)\r\n");
    if (json_mode) json["dynamixel_controller"] = RxByte;
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
    if (!json_mode) mySerCmd.Print((char *) "INFO: Arm Controller (v");
    if (!json_mode) mySerCmd.Print(RxByte);
    if (!json_mode) mySerCmd.Print((char *) ".0)\r\n");
    if (json_mode) json["arm_controller"] = RxByte;
  }

  if (json_mode) { serializeJson(json, Serial); Serial.println(); }
  sendOK();
}


// Set json mode disabled/enabled command. In json mode, responses are sent in JSON format
// Parameters
// int: active (0 = disable json mode, 1 = enable json mode)
// Example - "JSON,1"
void Set_Json(void) {
  JsonDocument json;
  uint8_t enableParam = 0;
  
  if (!mySerCmd.ReadNextUInt8(&enableParam)) {
    if (!json_mode) mySerCmd.Print((char *) "ERROR: Missing parameter\r\n");
    if (json_mode) json["success"] = "false";
    if (json_mode) json["command"] = "json";
    if (json_mode) json["error"] = "Missing parameter";
    if (json_mode) { serializeJson(json, Serial); Serial.println(); }
    return;
  }

  if (enableParam == 0) {
    json_mode = false;
    if (!json_mode) mySerCmd.Print((char *) "INFO: Json mode disabled\r\n");
  } else {
    json_mode = true;
    if (json_mode) json["success"] = "true";
    if (json_mode) json["command"] = "json";
    if (json_mode) json["value"] = "1";
  }

  if (json_mode) { serializeJson(json, Serial); Serial.println(); }
  sendOK();
}


// Set ToFs disabled/enabled command. If the ToFs are disabled the robot won't send a simulated bump command if the ToFs detect an object
// Parameters
// int: active (0 = disable tofs, 1 = enable tofs)
// Example - "TOFS,1"
void Set_Tofs(void) {
  JsonDocument json;
  uint8_t activeParam = 0;
  
  if (!mySerCmd.ReadNextUInt8(&activeParam)) {
    if (!json_mode) mySerCmd.Print((char *) "ERROR: Missing parameter\r\n");
    if (json_mode) json["success"] = "false";
    if (json_mode) json["command"] = "tofs";
    if (json_mode) json["error"] = "Missing parameter";
    if (json_mode) { serializeJson(json, Serial); Serial.println(); }
    return;
  }

  if (json_mode) json["success"] = "true";
  if (json_mode) json["command"] = "tofs";

  if (activeParam == 0) {
    tofs_active = false;
    if (!json_mode) mySerCmd.Print((char *) "INFO: Time of Flight sensors disabled\r\n");
    if (json_mode) json["value"] = "0";
  } else {
    tofs_active = true;
    if (!json_mode) mySerCmd.Print((char *) "INFO: Time of Flight sensors enabled\r\n");
    if (json_mode) json["value"] = "1";
  }

  if (json_mode) { serializeJson(json, Serial); Serial.println(); }
  sendOK();
}


// Initializes the connection between the Arduino on the Main Controller with the SLAM vacuum base robot. This must be run once after a power cycle before any other commands can be sent
// Example - "B_INIT"
void Send_Handshake(void) {
  byte response[88];

  if (!json_mode) mySerCmd.Print((char *) "INFO: Sending handshake_frame\r\n");
  Send_Frame_Get_Response(handshake_frame, sizeof(handshake_frame), response, sizeof(response));

  response[2] = 0x02;
  response[4] = 0x51;

  if (!json_mode) mySerCmd.Print((char *) "INFO: Sending handshake acknowledgement frame\r\n");
  Send_Frame(response, sizeof(response));

  Get_Packet(response[5]);

  sendOK();
}


// Set mode control to idle.
// Example - "B_MODE"
void Send_Mode(void) {
  uint8_t modeParam = 0;

  if (!mySerCmd.ReadNextUInt8(&modeParam)) {
    if (!json_mode) mySerCmd.Print((char *) "ERROR: Missing parameter\r\n");
    return;
  }

  // Constrain values to acceptable range and set the value into the frame
  modeParam = constrain(modeParam, 0, 12);
  mode_control_frame[7] = modeParam;

  if (!json_mode) mySerCmd.Print((char *) "INFO: Sending mode_control_frame\r\n");
  Send_Frame(mode_control_frame, sizeof(mode_control_frame));

  Get_Packet(mode_control_frame[5]);
  sendOK();
}


// Gets a list of all of the maps stored on the robot
// Example - "B_MAPLIST"
void Get_Maplist(void) {
  if (!json_mode) mySerCmd.Print((char *) "INFO: Sending get_map_list_frame\r\n");
  Send_Frame(get_map_list_frame, sizeof(get_map_list_frame));

  Get_Packet(get_map_list_frame[5]);
  sendOK();
}


// Set mode to starting mode. This moves the robot off the jsonk and starts the LiDAR
// Example - "B_START"
void Send_Start(void) {
  behavior_control_frame[7] = 0x00;

  if (!json_mode) mySerCmd.Print((char *) "INFO: Sending behavior_control_frame (ENTER)\r\n");
  Send_Frame(behavior_control_frame, sizeof(behavior_control_frame));

  Get_Packet(behavior_control_frame[5]);
  sendOK();
}


// Set mode to quick map. This moves the robot off the jsonk and generates an automatic map of your space
// Example - "B_QUICKMAP"
void Send_QuickMap(void) {
  behavior_control_frame[7] = 1;

  if (!json_mode) mySerCmd.Print((char *) "INFO: Sending behavior_control_frame (QUICKMAP)\r\n");
  Send_Frame(behavior_control_frame, sizeof(behavior_control_frame));

  Get_Packet(behavior_control_frame[5]);
  sendOK();
}


// Return to the charger jsonk and begin charging
// Example - "B_DOCK"
void Send_Dock(void) {
  behavior_control_frame[7] = 6;

  if (!json_mode) mySerCmd.Print((char *) "INFO: Sending behavior_control_frame (DOCK)\r\n");
  Send_Frame(behavior_control_frame, sizeof(behavior_control_frame));

  Get_Packet(behavior_control_frame[5]);
  sendOK();
}


// Send a stop command to the base
// Example - "B_KILL"
void Send_Kill(void) {
  behavior_control_frame[7] = 7;

  if (!json_mode) mySerCmd.Print((char *) "INFO: Sending behavior_control_frame (STOP)\r\n");
  Send_Frame(behavior_control_frame, sizeof(behavior_control_frame));

  Get_Packet(behavior_control_frame[5]);
  sendOK();
}


// Go to pose command. Must have a map created first.
// Parameters
// float: x (meters), float: y (meters), float: angle (degrees), float: speed (m/s)
// Example - "B_GOTO,0.5,0.5,0,10"
void Send_Goto(void) {
  float xParam = 0.0;
  float yParam = 0.0;
  float aParam = 0.0;
  float sParam = 0.0;

  if (!mySerCmd.ReadNextFloat(&xParam) || !mySerCmd.ReadNextFloat(&yParam) || !mySerCmd.ReadNextFloat(&aParam) || !mySerCmd.ReadNextFloat(&sParam)) {
    if (!json_mode) mySerCmd.Print((char *) "ERROR: Missing parameter\r\n");
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

  if (!json_mode) mySerCmd.Print((char *) "INFO: Sending behavior_control_frame (GOTO)\r\n");
  Send_Frame(behavior_control_frame, sizeof(behavior_control_frame));

  Get_Packet(behavior_control_frame[5]);
  sendOK();
}


// Simulated bumper command
// Parameters
// bool: left, bool: right
// Example - "B_BUMP,1,0"
void Send_Bump(void) {
  JsonDocument json;
  uint8_t leftParam = 0;
  uint8_t rightParam = 0;

  if (!mySerCmd.ReadNextUInt8(&leftParam) || !mySerCmd.ReadNextUInt8(&rightParam)) {
    if (!json_mode) mySerCmd.Print((char *) "ERROR: Missing parameter\r\n");
    if (json_mode) json["success"] = "false";
    if (json_mode) json["command"] = "bump";
    if (json_mode) json["error"] = "Missing parameter";
    if (json_mode) { serializeJson(json, Serial); Serial.println(); }
    return;
  }

  if (json_mode) json["success"] = "true";
  if (json_mode) json["command"] = "bump";

  // Constrain values to acceptable range and set the value into the frame
  leftParam = constrain(leftParam, 0, 1);
  rightParam = constrain(rightParam, 0, 1);

  // binary: 0, 0, 0, 0, 0, 0, left, right
  sim_bump_frame[7] = (leftParam << 1) | rightParam;

  if (!json_mode) mySerCmd.Print((char *) "INFO: Sending sim_bump_frame\r\n");
  Send_Frame(sim_bump_frame, sizeof(sim_bump_frame)/sizeof(sim_bump_frame[0]));

  if (json_mode) json["left"] = leftParam;
  if (json_mode) json["right"] = rightParam;

  if (json_mode) { serializeJson(json, Serial); Serial.println(); }
  sendOK();
}


// Wheel motor packet
// Parameters
// int16_t: linear_velocity (mm/s), int16_t: angular_velocity (degrees/s)
// Example - "B_DRIVE,10,20"
void Send_Drive(void) {
  JsonDocument json;
  float linearParam = 0.0;
  float angularParam = 0.0;

  int16_t linearParam16 = 0;
  int16_t angularParam16 = 0;

  if (!mySerCmd.ReadNextFloat(&linearParam) || !mySerCmd.ReadNextFloat(&angularParam)) {
    if (!json_mode) mySerCmd.Print((char *) "ERROR: Missing parameter\r\n");
    if (json_mode) json["success"] = "false";
    if (json_mode) json["command"] = "drive";
    if (json_mode) json["error"] = "Missing parameter";
    if (json_mode) { serializeJson(json, Serial); Serial.println(); }
    return;
  }

  if (json_mode) json["success"] = "true";
  if (json_mode) json["command"] = "drive";

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

  if (!json_mode) mySerCmd.Print((char *) "INFO: Sending wheel_motor_frame\r\n");
  Send_Frame(wheel_motor_frame, sizeof(wheel_motor_frame));

  if (json_mode) json["linear_velocity"] = "linearParam";
  if (json_mode) json["angular_velocity"] = "angularParam";

  if (json_mode) { serializeJson(json, Serial); Serial.println(); }
  sendOK();
}


// Get map command. Must have a map created first.
// Parameters
// int: map_id
// Example - "B_MAPDATA,2"
void Get_Mapdata(void) {
  JsonDocument json;
  uint8_t mapIdParam = 0;
  
  byte getMapFrameResponse[13];
  uint32_t currentCRC32;
  uint32_t lastCRC32;
  uint8_t doneFlag = 0;
  
  if (!mySerCmd.ReadNextUInt8(&mapIdParam)) {
    if (!json_mode) mySerCmd.Print((char *) "ERROR: Missing parameter\r\n");
    if (json_mode) json["success"] = "false";
    if (json_mode) json["command"] = "mapdata";
    if (json_mode) json["error"] = "Missing parameter";
    if (json_mode) { serializeJson(json, Serial); Serial.println(); }
    return;
  }

  // Constrain values to acceptable range and set the value into the frame
  mapIdParam = constrain(mapIdParam, 1, 255);
  get_map_frame[7] = mapIdParam;

  if (!json_mode) mySerCmd.Print((char *) "INFO: Sending get_map_frame\r\n");
  Send_Frame_Get_Response(get_map_frame, sizeof(get_map_frame), getMapFrameResponse, sizeof(getMapFrameResponse));

  if (getMapFrameResponse[7] == 0xFF && getMapFrameResponse[8] == 0xFF && getMapFrameResponse[9] == 0xFF && getMapFrameResponse[10] == 0xFF) {
    if (!json_mode) mySerCmd.Print((char *) "ERROR: Invalid map id!\r\n");
    if (json_mode) json["success"] = "false";
    if (json_mode) json["command"] = "mapdata";
    if (json_mode) json["error"] = "Invalid map id";
    if (json_mode) { serializeJson(json, Serial); Serial.println(); }
    return;
  }

  // Send CTRL_OTA_START_RESP packet
  if (!json_mode) mySerCmd.Print((char *) "INFO: Sending CTRL_OTA_START_RESP\r\n");
  Send_Frame(ctrl_ota_start_resp_frame, sizeof(ctrl_ota_start_resp_frame));
  Get_File_Transfer_Packet(0x10);


  // Send CTRL_OTA_FILE_INFO_RESP packet
  if (!json_mode) mySerCmd.Print((char *) "INFO: Sending CTRL_OTA_FILE_INFO_RESP\r\n");
  Send_Frame(ctrl_ota_file_info_resp_frame, sizeof(ctrl_ota_file_info_resp_frame));
  Get_File_Transfer_Packet(0x11);

  if (json_mode) mySerCmd.Print((char *) "{\"success\":\"true\",\"command\":\"mapdata\",\"compressedmapdata\":\"");

  // Send CTRL_OTA_FILE_POS_RESP packet
  if (!json_mode) mySerCmd.Print((char *) "INFO: Sending CTRL_OTA_FILE_POS_RESP\r\n");
  Send_Frame(ctrl_ota_file_pos_resp_frame, sizeof(ctrl_ota_file_pos_resp_frame));
  Get_File_Transfer_Packet(0x12, &currentCRC32);

  while (doneFlag == 0) {
    lastCRC32 = currentCRC32;

    ctrl_ota_file_data_resp_frame[5] = currentCRC32 & 0xFF;
    ctrl_ota_file_data_resp_frame[6] = (currentCRC32 >> 8) & 0xFF;
    ctrl_ota_file_data_resp_frame[7] = (currentCRC32 >> 16) & 0xFF;
    ctrl_ota_file_data_resp_frame[8] = (currentCRC32 >> 24) & 0xFF;

    // Send CTRL_OTA_FILE_DATA_RESP packet
    Send_Frame(ctrl_ota_file_data_resp_frame, sizeof(ctrl_ota_file_data_resp_frame));
    Get_File_Transfer_Packet(0x12, &currentCRC32, &lastCRC32, &doneFlag);
  }

  if (json_mode) mySerCmd.Print((char *) "\"}\r\n");

  sendOK();
}


// Get the latest saved synchonous packet
// Example - "B_STATUS"
void Get_Status(void) {
  char hexString[3];
  const char hexChars[] = "0123456789ABCDEF";

  if (!json_mode) mySerCmd.Print((char *) "INFO: Received    ");

  for (int i = 0; i < 27; i++) {
    hexString[0] = hexChars[synchronous_frame[i] >> 4];
    hexString[1] = hexChars[synchronous_frame[i] & 0x0F];
    hexString[2] = '\0';
    if (!json_mode) mySerCmd.Print(hexString);
    if (!json_mode) mySerCmd.Print((char *) " ");
  }

  if (!json_mode) mySerCmd.Print((char *) "\r\n");

  sendOK();
}


// Get the latest saved pose packet
// Example - "B_POSE"
void Get_Pose(void) {
  char hexString[3];
  const char hexChars[] = "0123456789ABCDEF";

  if (!json_mode) mySerCmd.Print((char *) "INFO: Received    ");

  for (int i = 0; i < 25; i++) {
    hexString[0] = hexChars[pose_frame[i] >> 4];
    hexString[1] = hexChars[pose_frame[i] & 0x0F];
    hexString[2] = '\0';
    if (!json_mode) mySerCmd.Print(hexString);
    if (!json_mode) mySerCmd.Print((char *) " ");
  }

  if (!json_mode) mySerCmd.Print((char *) "\r\n");

  sendOK();
}


// -------------------------------------------------------
// Head Functions
// -------------------------------------------------------
void set_H_IDLE(void) {
  char * sParam;
  sParam = mySerCmd.ReadNext();

  if (head_ame_attached == false) {
    if (!json_mode) mySerCmd.Print((char *) "ERROR: Dynamixel controller not attached\r\n");
    return;
  }
 
  if (sParam == NULL) {
    if (!json_mode) mySerCmd.Print((char *) "ERROR: Missing idle parameter\r\n");
    return;
  }

  if (strtoul(sParam, NULL, 10) == 0) {
    Wire.beginTransmission(DYN_I2C_ADDRESS);
    Wire.write(I2C_COMMAND_H_IDLE);
    Wire.write(0x00);
    Wire.endTransmission();
    if (!json_mode) mySerCmd.Print((char *) "INFO: Head idle mode disabled\r\n");
  } else {
    Wire.beginTransmission(DYN_I2C_ADDRESS);
    Wire.write(I2C_COMMAND_H_IDLE);
    Wire.write(0x01);
    Wire.endTransmission();
    if (!json_mode) mySerCmd.Print((char *) "INFO: Head idle mode enabled\r\n");
  }

  sendOK();
}


// Sets the position of the Hackerbot head's neck
// Parameters
// float: yaw (rotation angle between 100.0 and 260.0 degrees - 180.0 is looking straight ahead)
// float: pitch (vertical angle between 150.0 and 250.0 degrees - 180.0 is looking straight ahead)
// Example - "LOOK,180.0,180.0"
void set_H_LOOK(void) {
  float turnParam = 0.0;
  float vertParam = 0.0;
  uint8_t speedParam = 0;

  if (head_ame_attached == false) {
    if (!json_mode) mySerCmd.Print((char *) "ERROR: Dynamixel controller not attached\r\n");
    return;
  }
 
  if (!mySerCmd.ReadNextFloat(&turnParam) || !mySerCmd.ReadNextFloat(&vertParam) || !mySerCmd.ReadNextUInt8(&speedParam)) {
    if (!json_mode) mySerCmd.Print((char *) "ERROR: Missing parameter\r\n");
    return;
  }

  // Constrain values to acceptable range
  turnParam = constrain(turnParam, 100.0, 260.0);
  vertParam = constrain(vertParam, 150.0, 250.0);
  speedParam = constrain(speedParam, 6, 70);

  char buf[128] = {0};
  sprintf(buf, "INFO: Looking to position turn: %0.2f, vert: %0.2f, at speed: %d\r\n", turnParam, vertParam, speedParam);
  if (!json_mode) mySerCmd.Print(buf);

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


// Sets the gaze of the Hackerbot head's eyes
// Parameters
// float: x (position between -1.0 and 1.0)
// float: y (position between -1.0 and 1.0)
// Example - "GAZE,-0.8,0.2"
void set_H_GAZE(void) {
  float eyeTargetX = 0.0;
  float eyeTargetY = 0.0;

  if (head_ame_attached == false) {
    if (!json_mode) mySerCmd.Print((char *) "ERROR: Audio/Mouth/Eyes controller not attached\r\n");
    return;
  }

  if (!mySerCmd.ReadNextFloat(&eyeTargetX) || !mySerCmd.ReadNextFloat(&eyeTargetY)) {
    if (!json_mode) mySerCmd.Print((char *) "ERROR: Missing parameter\r\n");
    return;
  }

  // Constrain values to acceptable range
  eyeTargetX = constrain(eyeTargetX, -1.0, 1.0);
  eyeTargetY = constrain(eyeTargetY, -1.0, 1.0);

  char buf[128] = {0};
  sprintf(buf, "INFO: Setting: eyeTargetX: %0.2f, eyeTargetY: %0.2f\r\n", eyeTargetX, eyeTargetY);
  if (!json_mode) mySerCmd.Print(buf);

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
void run_A_CAL(void) {
  if (!json_mode) mySerCmd.Print((char *) "INFO: Calibrating the gripper\r\n");

  Wire.beginTransmission(ARM_I2C_ADDRESS);
  Wire.write(I2C_COMMAND_A_CAL);
  Wire.endTransmission();

  sendOK();
}


// A_OPEN
void set_A_OPEN(void) {
  if (!json_mode) mySerCmd.Print((char *) "INFO: Opening the gripper\r\n");

  Wire.beginTransmission(ARM_I2C_ADDRESS);
  Wire.write(I2C_COMMAND_A_OPEN);
  Wire.endTransmission();
  
  sendOK();
}


// A_CLOSE
void set_A_CLOSE(void) {
  if (!json_mode) mySerCmd.Print((char *) "INFO: Closing the gripper\r\n");

  Wire.beginTransmission(ARM_I2C_ADDRESS);
  Wire.write(I2C_COMMAND_A_CLOSE);
  Wire.endTransmission();
  
  sendOK();
}


// A_ANGLE
void set_A_ANGLE(void) {
  uint8_t jointParam = 0;
  float angleParam = 0.0;
  uint8_t speedParam = 0;

  if (!mySerCmd.ReadNextUInt8(&jointParam) || !mySerCmd.ReadNextFloat(&angleParam) || !mySerCmd.ReadNextUInt8(&speedParam)) {
    if (!json_mode) mySerCmd.Print((char *) "ERROR: Missing parameter\r\n");
    return;
  }

  jointParam = constrain(jointParam, 1, 6);

  if (jointParam != 6) {
    angleParam = constrain(angleParam, -165.0, 165);
  } else {
    angleParam = constrain(angleParam, -175.0, 175);
  }

  speedParam  = constrain(speedParam, 0, 100);

  if (!json_mode) mySerCmd.Print((char *) "STATUS: Setting the angle of joint ");
  if (!json_mode) mySerCmd.Print((int)jointParam);
  if (!json_mode) mySerCmd.Print((char *) " to ");
  if (!json_mode) mySerCmd.Print(angleParam);
  if (!json_mode) mySerCmd.Print((char *) " degrees at speed ");
  if (!json_mode) mySerCmd.Print((int)speedParam);
  if (!json_mode) mySerCmd.Print((char *) "\r\n");

  uint8_t jointParam8 = (uint8_t)(jointParam);
  uint16_t angleParam16 = (uint16_t)((angleParam + 165.0) * 10);
  uint8_t speedParam8 = (uint8_t)(speedParam);

  Wire.beginTransmission(ARM_I2C_ADDRESS);
  Wire.write(I2C_COMMAND_A_ANGLE);
  Wire.write(jointParam8);
  Wire.write(highByte(angleParam16)); // Angle H
  Wire.write(lowByte(angleParam16)); // Angle L
  Wire.write(speedParam8);
  Wire.endTransmission();

  sendOK();
}


// A_ANGLES
void set_A_ANGLES(void) {
  float joint1Param = 0.0;
  float joint2Param = 0.0;
  float joint3Param = 0.0;
  float joint4Param = 0.0;
  float joint5Param = 0.0;
  float joint6Param = 0.0;
  uint8_t speedParam = 0;

  if (!mySerCmd.ReadNextFloat(&joint1Param) || 
      !mySerCmd.ReadNextFloat(&joint2Param) || 
      !mySerCmd.ReadNextFloat(&joint3Param) || 
      !mySerCmd.ReadNextFloat(&joint4Param) || 
      !mySerCmd.ReadNextFloat(&joint5Param) || 
      !mySerCmd.ReadNextFloat(&joint6Param) || 
      !mySerCmd.ReadNextUInt8(&speedParam)) {
    if (!json_mode) mySerCmd.Print((char *) "ERROR: Missing parameter\r\n");
    return;
  }

  joint1Param = constrain(joint1Param, -165.0, 165);
  joint2Param = constrain(joint2Param, -165.0, 165);
  joint3Param = constrain(joint3Param, -165.0, 165);
  joint4Param = constrain(joint4Param, -165.0, 165);
  joint5Param = constrain(joint5Param, -165.0, 165);
  joint6Param = constrain(joint6Param, -175.0, 175);
  speedParam  = constrain(speedParam, 0, 100);

  if (!json_mode) mySerCmd.Print((char *) "STATUS: Setting the angle of the joints to (1) ");
  if (!json_mode) mySerCmd.Print(joint1Param);
  if (!json_mode) mySerCmd.Print((char *) ", (2) ");
  if (!json_mode) mySerCmd.Print(joint2Param);
  if (!json_mode) mySerCmd.Print((char *) ", (3) ");
  if (!json_mode) mySerCmd.Print(joint3Param);
  if (!json_mode) mySerCmd.Print((char *) ", (4) ");
  if (!json_mode) mySerCmd.Print(joint4Param);
  if (!json_mode) mySerCmd.Print((char *) ", (5) ");
  if (!json_mode) mySerCmd.Print(joint5Param);
  if (!json_mode) mySerCmd.Print((char *) ", (6) ");
  if (!json_mode) mySerCmd.Print(joint6Param);
  if (!json_mode) mySerCmd.Print((char *) " degrees at speed ");
  if (!json_mode) mySerCmd.Print((int)speedParam);
  if (!json_mode) mySerCmd.Print((char *) "\r\n");

  uint16_t joint1Param16 = (uint16_t)((joint1Param + 165.0) * 10);
  uint16_t joint2Param16 = (uint16_t)((joint2Param + 165.0) * 10);
  uint16_t joint3Param16 = (uint16_t)((joint3Param + 165.0) * 10);
  uint16_t joint4Param16 = (uint16_t)((joint4Param + 165.0) * 10);
  uint16_t joint5Param16 = (uint16_t)((joint5Param + 165.0) * 10);
  uint16_t joint6Param16 = (uint16_t)((joint6Param + 175.0) * 10);
  uint8_t speedParam8 = (uint8_t)(speedParam);

  Wire.beginTransmission(ARM_I2C_ADDRESS);
  Wire.write(I2C_COMMAND_A_ANGLES);
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


// -------------------------------------------------------
// General Helper Functions
// -------------------------------------------------------
void Get_Packet(byte response_packetid, byte response_frame[], int sizeOfResponseFrame) {
  JsonDocument json;
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
            if (!json_mode) mySerCmd.Print((char *) "WARNING: Timed out while waiting for the first header byte in the response packet (0x55)\r\n");
            if (json_mode) json["success"] = "false";
            if (json_mode) json["error"] = "Timed out while waiting for the first header byte in the response packet (0x55)";
            if (json_mode) { serializeJson(json, Serial); Serial.println(); }
            return;
        }
      }

      incomingByte = Serial1.read();
      incomingPacket[incomingPacketLen] = incomingByte;

      if (incomingPacket[incomingPacketLen] != 0x55) {
        if (responseByteReceived) {
          if (!json_mode) mySerCmd.Print((char *) "WARNING: Unexpected byte received (not 0x55) ");
          hexString[0] = hexChars[incomingPacket[incomingPacketLen] >> 4];
          hexString[1] = hexChars[incomingPacket[incomingPacketLen] & 0x0F];
          hexString[2] = '\0';
          if (!json_mode) mySerCmd.Print(hexString);
          if (!json_mode) mySerCmd.Print((char *) "\r\n");
          if (json_mode) json["success"] = "false";
          if (json_mode) json["error"] = "WARNING: Unexpected byte received (not 0x55)";
          if (json_mode) { serializeJson(json, Serial); Serial.println(); }
        }
      } else {
        incomingPacketLen++;
        
        while (!Serial1.available()) {
          if (millis() - responseTimeout >= response_timeout_period) {
            if (!json_mode) mySerCmd.Print((char *) "WARNING: Timed out while waiting for the second header byte in the response packet (0xAA)\r\n");
            if (json_mode) json["success"] = "false";
            if (json_mode) json["error"] = "Timed out while waiting for the second header byte in the response packet (0xAA)";
            if (json_mode) { serializeJson(json, Serial); Serial.println(); }
            return;
          }
        }

        incomingByte = Serial1.read();
        incomingPacket[incomingPacketLen] = incomingByte;

        if(incomingPacket[incomingPacketLen] != 0xAA) {
          if (responseByteReceived) {
            if (!json_mode) mySerCmd.Print((char *) "WARNING: Unexpected byte received (not 0xAA) ");
            hexString[0] = hexChars[incomingPacket[incomingPacketLen] >> 4];
            hexString[1] = hexChars[incomingPacket[incomingPacketLen] & 0x0F];
            hexString[2] = '\0';
            if (!json_mode) mySerCmd.Print(hexString);
            if (!json_mode) mySerCmd.Print((char *) "\r\n");
            if (json_mode) json["success"] = "false";
            if (json_mode) json["error"] = "Unexpected byte received (not 0xAA)";
            if (json_mode) { serializeJson(json, Serial); Serial.println(); }
          }

          incomingPacketLen = 0;
        }
      }
    }

    incomingPacketLen++;

    // Wait for and receive the CTRL_ID
    while(!Serial1.available()) {
      if(millis() - responseTimeout >= response_timeout_period) {
          if (!json_mode) mySerCmd.Print((char *) "WARNING: Timed out while waiting for the CTRL_ID in the response packet\r\n");
          if (json_mode) json["success"] = "false";
          if (json_mode) json["error"] = "Timed out while waiting for the CTRL_ID in the response packet";
          if (json_mode) { serializeJson(json, Serial); Serial.println(); }
          return;
      }
    }

    incomingByte = Serial1.read();
    incomingPacket[incomingPacketLen] = incomingByte;
    incomingPacketLen++;

    // Wait for and receive the LEN_HI
    while(!Serial1.available()) {
      if(millis() - responseTimeout >= response_timeout_period) {
          if (!json_mode) mySerCmd.Print((char *) "WARNING: Timed out while waiting for the LEN_HI in the response packet\r\n");
          if (json_mode) json["success"] = "false";
          if (json_mode) json["error"] = "Timed out while waiting for the LEN_HI in the response packet";
          if (json_mode) { serializeJson(json, Serial); Serial.println(); }
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
          if (!json_mode) mySerCmd.Print((char *) "WARNING: Timed out while waiting for the LEN_LOW in the response packet\r\n");
          if (json_mode) json["success"] = "false";
          if (json_mode) json["error"] = "Timed out while waiting for the LEN_LOW in the response packet";
          if (json_mode) { serializeJson(json, Serial); Serial.println(); }
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
          if (!json_mode) mySerCmd.Print((char *) "WARNING: Timed out while waiting for the PACKET_ID in the response packet\r\n");
          if (json_mode) json["success"] = "false";
          if (json_mode) json["error"] = "Timed out while waiting for the PACKET_ID in the response packet";
          if (json_mode) { serializeJson(json, Serial); Serial.println(); }
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
            if (!json_mode) mySerCmd.Print((char *) "WARNING: Timed out while waiting for the DATA or CRC in the response packet\r\n");
            if (json_mode) json["success"] = "false";
            if (json_mode) json["error"] = "Timed out while waiting for the DATA or CRC in the response packet";
            if (json_mode) { serializeJson(json, Serial); Serial.println(); }
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
      if (!json_mode) mySerCmd.Print((char *) "WARNING: Packet length is too long for the incomingPacket buffer. Throwing out ");
      if (!json_mode) mySerCmd.Print(lenByte + 7);
      if (!json_mode) mySerCmd.Print((char *) " bytes from PACKET_ID 0x");
      hexString[0] = hexChars[packetID >> 4];
      hexString[1] = hexChars[packetID & 0x0F];
      hexString[2] = '\0';
      if (!json_mode) mySerCmd.Print(hexString);
      if (!json_mode) mySerCmd.Print((char *) "\r\n");
      if (json_mode) json["success"] = "false";
      if (json_mode) json["error"] = "Packet length is too long for the incomingPacket buffer. Throwing out data.";
      if (json_mode) { serializeJson(json, Serial); Serial.println(); }
      return;
    }

    // Don't display...
    // Status Packets = 0x02
    // Pose Packets = 0x23
    if ((incomingPacket[5] == 0x02 || incomingPacket[5] == 0x23) && responseByteReceived) {
      if (incomingPacket[5] == 0x02) {
        memcpy(synchronous_frame, incomingPacket, 27);
      }

      if (incomingPacket[5] == 0x23) {
        memcpy(pose_frame, incomingPacket, 25);
      }

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

  if (!json_mode) mySerCmd.Print((char *) "INFO: Received    ");
  for (int i = 0; i < incomingPacketLen; i++) {
    if (response_frame != nullptr) {
      if (i < sizeOfResponseFrame) {
        response_frame[i] = incomingPacket[i];
      }
    }
    hexString[0] = hexChars[incomingPacket[i] >> 4];
    hexString[1] = hexChars[incomingPacket[i] & 0x0F];
    hexString[2] = '\0';
    if (!json_mode) mySerCmd.Print(hexString);
    if (!json_mode) mySerCmd.Print((char *) " ");
  }
  if (!json_mode) mySerCmd.Print((char *) " (");
  if (!json_mode) mySerCmd.Print(incomingPacketLen);
  if (!json_mode) mySerCmd.Print((char *) ")\r\n");
  if (json_mode) { Generate_json_mode_Json(incomingPacket, incomingPacketLen); }

  if (response_frame != nullptr) {
    if (incomingPacketLen != sizeOfResponseFrame) {
      if (!json_mode) mySerCmd.Print((char *) "WARNING: Length of the frame received does not match the length that was specified\r\n");
      if (json_mode) json["success"] = "false";
      if (json_mode) json["error"] = "Length of the frame received does not match the length that was specified";
      if (json_mode) { serializeJson(json, Serial); Serial.println(); }
    }
  }
}


// Get the request packet for a file transfer
void Get_File_Transfer_Packet(byte request_ctrlid, uint32_t* currentCRC32, uint32_t* lastCRC32, uint8_t* doneFlag) {
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
            if (!json_mode) mySerCmd.Print((char *) "WARNING: Timed out while waiting for the first header byte in the response packet (0x55)\r\n");
            return;
        }
      }

      incomingByte = Serial1.read();
      incomingPacket[incomingPacketLen] = incomingByte;

      if (incomingPacket[incomingPacketLen] != 0x55) {
        if (requestByteReceived) {
          if (!json_mode) mySerCmd.Print((char *) "WARNING: Unexpected byte received (not 0x55) ");
          hexString[0] = hexChars[incomingPacket[incomingPacketLen] >> 4];
          hexString[1] = hexChars[incomingPacket[incomingPacketLen] & 0x0F];
          hexString[2] = '\0';
          if (!json_mode) mySerCmd.Print(hexString);
          if (!json_mode) mySerCmd.Print((char *) "\r\n");
        }
      } else {
        incomingPacketLen++;
        
        while (!Serial1.available()) {
          if (millis() - responseTimeout >= response_timeout_period) {
            if (!json_mode) mySerCmd.Print((char *) "WARNING: Timed out while waiting for the second header byte in the response packet (0xAA)\r\n");
            return;
          }
        }

        incomingByte = Serial1.read();
        incomingPacket[incomingPacketLen] = incomingByte;

        if(incomingPacket[incomingPacketLen] != 0xAA) {
          if (requestByteReceived) {
            if (!json_mode) mySerCmd.Print((char *) "WARNING: Unexpected byte received (not 0xAA) ");
            hexString[0] = hexChars[incomingPacket[incomingPacketLen] >> 4];
            hexString[1] = hexChars[incomingPacket[incomingPacketLen] & 0x0F];
            hexString[2] = '\0';
            if (!json_mode) mySerCmd.Print(hexString);
            if (!json_mode) mySerCmd.Print((char *) "\r\n");
          }

          incomingPacketLen = 0;
        }
      }
    }

    incomingPacketLen++;

    // Wait for and receive the CTRL_ID
    while(!Serial1.available()) {
      if(millis() - responseTimeout >= response_timeout_period) {
          if (!json_mode) mySerCmd.Print((char *) "WARNING: Sending frame timed out while waiting for the CTRL_ID in the response packet\r\n");
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
          if (!json_mode) mySerCmd.Print((char *) "WARNING: Sending frame timed out while waiting for the LEN_HI in the response packet\r\n");
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
          if (!json_mode) mySerCmd.Print((char *) "WARNING: Sending frame timed out while waiting for the LEN_LOW in the response packet\r\n");
          return;
      }
    }

    incomingByte = Serial1.read();
    incomingPacket[incomingPacketLen] = incomingByte;
    lenByteLow = incomingPacket[incomingPacketLen];
    incomingPacketLen++;

    lenByte = (lenByteHigh << 8) | lenByteLow;

    // File end packet
    if (ctrlID == 0x13) {
      *doneFlag = 1;
      if (!json_mode) mySerCmd.Print((char *) "\r\n");
    }

    // Wait for and receive the remaining packet data
    for (int i = 0; i < (lenByte + 2); i++) {
      while(!Serial1.available()) {
        if(millis() - responseTimeout >= response_timeout_period) {
            if (!json_mode) mySerCmd.Print((char *) "WARNING: Sending frame timed out while waiting for the DATA or CRC in the response packet\r\n");
            return;
        }
      }

      if (lenByte <= (incomingPacketBuffSize - 5 - 2)) {
        incomingByte = Serial1.read();
        incomingPacket[incomingPacketLen] = incomingByte;
        incomingPacketLen++;
      } else {
        incomingByte = Serial1.read();
      }
    }

    if (lenByte > (incomingPacketBuffSize - 5 - 2)) {
      if (!json_mode) mySerCmd.Print((char *) "WARNING: Packet length is too long for the incomingPacket buffer. Throwing out ");
      if (!json_mode) mySerCmd.Print(lenByte + 7);
      if (!json_mode) mySerCmd.Print((char *) " bytes from CTRL_ID 0x");
      hexString[0] = hexChars[ctrlID >> 4];
      hexString[1] = hexChars[ctrlID & 0x0F];
      hexString[2] = '\0';
      if (!json_mode) mySerCmd.Print(hexString);
      if (!json_mode) mySerCmd.Print((char *) "\r\n");
      return;
    }

    if ((incomingPacket[2] == request_ctrlid) || (incomingPacket[2] == 0x13)) {
      requestByteReceived = true;
    } else {
      incomingPacket[0] = 0x00;
      incomingPacket[1] = 0x00;
      incomingPacketLen = 0;
    }
  }

  if (incomingPacket[2] != 0x12) {
    if (!json_mode) mySerCmd.Print((char *) "INFO: Received    ");
    for (int i = 0; i < incomingPacketLen; i++) {
      hexString[0] = hexChars[incomingPacket[i] >> 4];
      hexString[1] = hexChars[incomingPacket[i] & 0x0F];
      hexString[2] = '\0';
      if (!json_mode) mySerCmd.Print(hexString);
      if (!json_mode) mySerCmd.Print((char *) " ");
    }
    if (!json_mode) mySerCmd.Print((char *) " (");
    if (!json_mode) mySerCmd.Print(incomingPacketLen);
    if (!json_mode) mySerCmd.Print((char *) ")\r\n");
  } else {
    //if (!json_mode) mySerCmd.Print((char *) "INFO: FP Received ");
    for (int i = 7; i < incomingPacketLen - 2; i++) {
      hexString[0] = hexChars[incomingPacket[i] >> 4];
      hexString[1] = hexChars[incomingPacket[i] & 0x0F];
      hexString[2] = '\0';
      //if (!json_mode) mySerCmd.Print(hexString);
      mySerCmd.Print(hexString);
      //if (json_mode) Serial.print(hexString);
      //if (!json_mode) mySerCmd.Print((char *) " ");
    }
  }

  if (incomingPacket[2] == 0x12 && currentCRC32 != nullptr) {
    uint32_t crc32;

    if (lastCRC32 == nullptr) {
      crc32 = crc32_compute(&incomingPacket[7], incomingPacketLen - 9, NULL);
    } else {
      crc32 = crc32_compute(&incomingPacket[7], incomingPacketLen - 9, lastCRC32);
    }
    *currentCRC32 = crc32;
  }
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

  if (frame[2] != 0x32) {
    if (!json_mode) mySerCmd.Print((char *) "INFO: Transmitted ");
    for(int i = 0; i < sizeOfFrame; i++) {
      hexString[0] = hexChars[frame[i] >> 4];
      hexString[1] = hexChars[frame[i] & 0x0F];
      hexString[2] = '\0';
      if (!json_mode) mySerCmd.Print(hexString);
      if (!json_mode) mySerCmd.Print((char *) " ");
    }
    if (!json_mode) mySerCmd.Print((char *) "\r\n");
  }

  Serial1.write(frame, sizeOfFrame);
}

// Sends a frame and then saves the response frame with the matching packet id
void Send_Frame_Get_Response(byte frame[], int sizeOfFrame, byte response_frame[], int sizeOfResponseFrame) {
  Send_Frame(frame, sizeOfFrame);
  Get_Packet(frame[5], response_frame, sizeOfResponseFrame);
}


void Generate_json_mode_Json(byte response_frame[], int sizeOfResponseFrame) {
  JsonDocument json;

  uint8_t packet_id;
  
  if (sizeOfResponseFrame < 9 || response_frame[0] != 0x55 || response_frame[1] != 0xAA || response_frame[2] != 0x03) {
    json["success"] = "false";
    json["error"] = "Invalid response packet received";
    serializeJson(json, Serial);
    Serial.println();
    return;
  }

  packet_id = response_frame[5];

  switch (packet_id) {
    case 0x20: {
      json["success"] = "true";
      json["command"] = "maplist";
      json["map_num"] = response_frame[7];

      JsonArray map_ids = json["map_ids"].to<JsonArray>();
      for (int i = 8; i < (8 + (response_frame[7] * 4)); i += 4) {
        int value = (response_frame[i]) | (response_frame[i + 1] << 8) | (response_frame[i + 2] << 16) | (response_frame[i + 3] << 24);
        map_ids.add(value);
      }
      break;
    }

    case 0x21:
      return;
      break;

    default: 
      json["success"] = "false";
      json["error"] = "Unsupported packet id";
      break;
  }

  serializeJson(json, Serial);
  Serial.println();
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