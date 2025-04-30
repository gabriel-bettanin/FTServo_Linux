/*
 * SCSCL.h
 * Feit SCSCL series serial servo application layer program
 * Date: 2020.6.17
 * Author.
 */

#ifndef _SCSCL_H
#define _SCSCL_H

#define SCSCL_1M 0
#define SCSCL_0_5M 1
#define SCSCL_250K 2
#define SCSCL_128K 3
#define SCSCL_115200 4
#define SCSCL_76800 5
#define SCSCL_57600 6
#define SCSCL_38400 7

// Memory table definition
//-------EPROM (Read-only)--------
#define SCSCL_VERSION_L 3
#define SCSCL_VERSION_H 4

//-------EPROM(Read/Write)--------
#define SCSCL_ID 5
#define SCSCL_BAUD_RATE 6
#define SCSCL_MIN_ANGLE_LIMIT_L 9
#define SCSCL_MIN_ANGLE_LIMIT_H 10
#define SCSCL_MAX_ANGLE_LIMIT_L 11
#define SCSCL_MAX_ANGLE_LIMIT_H 12
#define SCSCL_CW_DEAD 26
#define SCSCL_CCW_DEAD 27

//-------SRAM(Read/Write)--------
#define SCSCL_TORQUE_ENABLE 40
#define SCSCL_GOAL_POSITION_L 42
#define SCSCL_GOAL_POSITION_H 43
#define SCSCL_GOAL_TIME_L 44
#define SCSCL_GOAL_TIME_H 45
#define SCSCL_GOAL_SPEED_L 46
#define SCSCL_GOAL_SPEED_H 47
#define SCSCL_LOCK 48

//-------SRAM(Read-only)--------
#define SCSCL_PRESENT_POSITION_L 56
#define SCSCL_PRESENT_POSITION_H 57
#define SCSCL_PRESENT_SPEED_L 58
#define SCSCL_PRESENT_SPEED_H 59
#define SCSCL_PRESENT_LOAD_L 60
#define SCSCL_PRESENT_LOAD_H 61
#define SCSCL_PRESENT_VOLTAGE 62
#define SCSCL_PRESENT_TEMPERATURE 63
#define SCSCL_MOVING 66
#define SCSCL_PRESENT_CURRENT_L 69
#define SCSCL_PRESENT_CURRENT_H 70

#include "SCSerial.h"

class SCSCL : public SCSerial
{
public:
  SCSCL();
  SCSCL(u8 End);
  SCSCL(u8 End, u8 Level);
  virtual int writePos(u8 ID, u16 Position, u16 Time, u16 Speed = 0);                   // Write position command for a single servo
  virtual int regWritePos(u8 ID, u16 Position, u16 Time, u16 Speed = 0);                // Asynchronous write position command for a single servo (takes effect with RegWriteAction)
  virtual void syncWritePos(u8 ID[], u8 IDN, u16 Position[], u16 Time[], u16 Speed[]);  // Synchronous write position command for multiple servos
  virtual int pwmMode(u8 ID);                                                           // Set PWM output mode
  virtual int writePWM(u8 ID, s16 pwmOut);                                              // Command for PWM output mode
  virtual int enableTorque(u8 ID, u8 Enable);                                           // Torque control command
  virtual int unlockEprom(u8 ID);                                                       // Unlock EEPROM
  virtual int lockEprom(u8 ID);                                                         // Lock EEPROM
  virtual int feedback(int ID);                                                         // Get feedback from servo
  virtual int readPos(int ID);                                                          // Read position
  virtual int readSpeed(int ID);                                                        // Read speed
  virtual int readLoad(int ID);                                                         // Read load (percentage of voltage output to motor, 0~1000)
  virtual int readVoltage(int ID);                                                      // Read voltage
  virtual int readTemperature(int ID);                                                  // Read temperature
  virtual int readMove(int ID);                                                         // Read movement state
  virtual int readCurrent(int ID);                                                      // Read current
private:
  u8 Mem[SCSCL_PRESENT_CURRENT_H - SCSCL_PRESENT_POSITION_L + 1];
};

#endif