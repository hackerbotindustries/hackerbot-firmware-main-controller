/*********************************************************************************
Hackerbot Industries, LLC
Created: April 2024
Updated: 2024.11.11

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

#define SERIALCMD_MAXCMDNUM 30    // Max number of commands
#define SERIALCMD_MAXCMDLNG 12     // Max command name length
#define SERIALCMD_MAXBUFFER 256    // Max buffer length

SerialCmd mySerCmd(Serial);       // Initialize the SerialCmd constructor using the "Serial" port

Adafruit_NeoPixel onboard_pixel(1, PIN_NEOPIXEL);

// ------------------- User functions --------------------
void sendOK(void) {
  mySerCmd.Print((char *) "OK\r\n");
}


// --------------- Functions for SerialCmd ---------------
void Send_Ping(void) {
  mySerCmd.Print((char *) "INFO: Ping\r\n");
  sendOK();
}

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


// Get map command - GETMAP1
// Example - "GETMAP1"
void Get_Map1(void) {
  byte get_map1_frame[] = {
    0x55, 0xAA, // HEADER_HI, HEADER_LOW
    0x02, // CTRL_ID
    0x00, 0x06, // LEN_HI, LEN_LOW
    0x21, // PACKET_ID
    0x04, // PACKET_LEN
    0x03, 0x00, 0x00, 0x00, // map_id
    0x05, 0x30 // CRC_HI, CRC_LOW
  };

  mySerCmd.Print((char *) "Sending get_map1_frame\r\n");
  Serial1.write(get_map1_frame, sizeof(get_map1_frame));

  for(int i = 0; i < 8; i++) {
    while(!Serial1.available());
    byte incomingByte = Serial1.read();
    String incomingHex = String(incomingByte, HEX);
    incomingHex.toUpperCase();
    mySerCmd.Print(incomingHex);
    mySerCmd.Print((char *) " ");
  }
  mySerCmd.Print((char *) "\r\n");

  mySerCmd.Print((char *) "Sending get_map2_frame\r\n");
  Get_Map2();

  for(int i = 0; i < 39; i++) {
    while(!Serial1.available());
    byte incomingByte = Serial1.read();
    String incomingHex = String(incomingByte, HEX);
    incomingHex.toUpperCase();
    mySerCmd.Print(incomingHex);
    mySerCmd.Print((char *) " ");
  }
  mySerCmd.Print((char *) "\r\n");

  mySerCmd.Print((char *) "Sending get_map_frame\r\n");
  Get_Map3();

  for(int i = 0; i < 11; i++) {
    while(!Serial1.available());
    byte incomingByte = Serial1.read();
    String incomingHex = String(incomingByte, HEX);
    incomingHex.toUpperCase();
    mySerCmd.Print(incomingHex);
    mySerCmd.Print((char *) " ");
  }
  mySerCmd.Print((char *) "\r\n");

  mySerCmd.Print((char *) "Sending get_map4_frame\r\n");
  Get_Map4();

  sendOK();
}

// Send CTRL_OTA_START_RESP packet - GETMAP2
// Example - "GETMAP2"
void Get_Map2(void) {
  byte get_map2_frame[] = {
    0x55, 0xAA, // HEADER_HI, HEADER_LOW
    0x2F, // CTRL_ID_RESP
    0x00, 0x17, // LEN_HI, LEN_LOW
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // no_use0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // no_use1
    0x00, 0x28, 0x00, 0x00, // file_limit
    0x7F, 0x00, // packet_size
    0x00, // no_use2
    0x81, 0x74 // CRC_HI, CRC_LOW
  };
  Serial1.write(get_map2_frame, sizeof(get_map2_frame));

  //sendOK();
}

// Send CTRL_OTA_FILE_INFO_RESP packet - GETMAP3
// Example - "GETMAP3"
void Get_Map3(void) {
  byte get_map3_frame[] = {
    0x55, 0xAA, // HEADER_HI, HEADER_LOW
    0x30, // CTRL_ID_RESP
    0x00, 0x09, // LEN_HI, LEN_LOW
    0x00, // stat
    0x00, 0x00, 0x00, 0x00, // last_offset
    0x00, 0x00, 0x00, 0x00, // last_crc32
    0x7C, 0xFE // CRC_HI, CRC_LOW
  };
  Serial1.write(get_map3_frame, sizeof(get_map3_frame));

  //sendOK();
}

// Send CTRL_OTA_FILE_POS_RESP packet - GETMAP4
// Example - "GETMAP4"
void Get_Map4(void) {
  byte get_map4_frame[] = {
    0x55, 0xAA, // HEADER_HI, HEADER_LOW
    0x31, // CTRL_ID_RESP
    0x00, 0x04, // LEN_HI, LEN_LOW
    0x00, 0x00, 0x00, 0x00, // last_offset
    0x68, 0xEA // CRC_HI, CRC_LOW
  };
  Serial1.write(get_map4_frame, sizeof(get_map4_frame));

  //sendOK();
}

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

// Go to pose command - GOTO
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


// Go to dock command - DOCK
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

// Simulate bumper - BUMP
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

// Wheel motor packet - MOTOR
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


// ----------------------- setup() -----------------------
void setup() {
  Serial.begin(115200);
  //while(!Serial);
  Serial1.begin(230400);

  onboard_pixel.begin();
  onboard_pixel.setPixelColor(0, onboard_pixel.Color(0, 0, 10));
  onboard_pixel.show();

  // Command Setup
  mySerCmd.AddCmd("PING", SERIALCMD_FROMALL, Send_Ping);
  mySerCmd.AddCmd("INIT", SERIALCMD_FROMALL, Send_Handshake);
  //mySerCmd.AddCmd("GETML", SERIALCMD_FROMALL, Get_MapList);
  //mySerCmd.AddCmd("GETMAP1", SERIALCMD_FROMALL, Get_Map1);
  //mySerCmd.AddCmd("GETMAP2", SERIALCMD_FROMALL, Get_Map2);
  //mySerCmd.AddCmd("GETMAP3", SERIALCMD_FROMALL, Get_Map3);
  //mySerCmd.AddCmd("GETMAP4", SERIALCMD_FROMALL, Get_Map4);
  mySerCmd.AddCmd("ENTER", SERIALCMD_FROMALL, Send_Enter);
  mySerCmd.AddCmd("GOTO", SERIALCMD_FROMALL, Send_Goto);
  mySerCmd.AddCmd("DOCK", SERIALCMD_FROMALL, Send_Dock);
  mySerCmd.AddCmd("MOTOR", SERIALCMD_FROMALL, Send_Motor);

  // Setup
  mySerCmd.Print((char *) "INFO: Starting application...\r\n");
}


// ----------------------- loop() ------------------------
void loop() {
  byte incomingByte;
  byte incomingPacket[100];
  int incomingPacketLen;
  byte lenByte;
  int8_t ret;

  // Check for incoming serial commands
  ret = mySerCmd.ReadSer();
  if (ret == 0) {
    mySerCmd.Print((char *) "ERROR: Urecognized command.\r\n");
  }

  // Check for data coming from the rc330b
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