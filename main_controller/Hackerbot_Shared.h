/*********************************************************************************
Hackerbot Industries, LLC
Created: February 2025

A shared library of defines and helper functions used across all of the Hackerbot
controller PCBs (main, dynamixel, audio/mouth/eyes, eyes, and arm)
*********************************************************************************/

#ifndef HACKERBOTSHARED_H
#define HACKERBOTSHARED_H

// Hackerbot controller PCBA I2C addresses
#define AME_I2C_ADDRESS 0x5A              // Audio Mouth Eyes PCBA I2C address
#define DYN_I2C_ADDRESS 0x5B              // Dynamixel Controller I2C address
#define ARM_I2C_ADDRESS 0x5C              // Arm Controller I2C address

// I2C command addresses
#define I2C_COMMAND_PING        0x01
#define I2C_COMMAND_VERSION     0x02
#define I2C_COMMAND_H_IDLE      0x08
#define I2C_COMMAND_H_LOOK      0x09
#define I2C_COMMAND_H_GAZE      0x0A
#define I2C_COMMAND_A_CAL       0x20
#define I2C_COMMAND_A_OPEN      0x21
#define I2C_COMMAND_A_CLOSE     0x22
#define I2C_COMMAND_A_ANGLE     0x25
#define I2C_COMMAND_A_ANGLES    0x26


#endif