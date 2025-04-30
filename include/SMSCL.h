/*
 * SMSCL.h
 * ����SMSCLϵ�д��ж���ӿ�
 * ����: 2020.6.17
 * ����: 
 */

#ifndef _SMSCL_H
#define _SMSCL_H

#define SMSCL_1M 0
#define SMSCL_0_5M 1
#define SMSCL_250K 2
#define SMSCL_128K 3
#define SMSCL_115200 4
#define SMSCL_76800 5
#define SMSCL_57600 6
#define SMSCL_38400 7

// Memory table definitions
//-------EPROM(Read-only)--------
#define SMSCL_VERSION_L 3
#define SMSCL_VERSION_H 4

// -------EPROM (Read/Write)--------
#define SMSCL_ID 5
#define SMSCL_BAUD_RATE 6
#define SMSCL_RETURN_DELAY_TIME 7
#define SMSCL_RETURN_LEVEL 8
#define SMSCL_MIN_ANGLE_LIMIT_L 9
#define SMSCL_MIN_ANGLE_LIMIT_H 10
#define SMSCL_MAX_ANGLE_LIMIT_L 11
#define SMSCL_MAX_ANGLE_LIMIT_H 12
#define SMSCL_LIMIT_TEMPERATURE 13
#define SMSCL_MAX_LIMIT_VOLTAGE 14
#define SMSCL_MIN_LIMIT_VOLTAGE 15
#define SMSCL_MAX_TORQUE_L 16
#define SMSCL_MAX_TORQUE_H 17
#define SMSCL_ALARM_LED 19
#define SMSCL_ALARM_SHUTDOWN 20
#define SMSCL_COMPLIANCE_P 21
#define SMSCL_COMPLIANCE_D 22
#define SMSCL_COMPLIANCE_I 23
#define SMSCL_PUNCH_L 24
#define SMSCL_PUNCH_H 25
#define SMSCL_CW_DEAD 26
#define SMSCL_CCW_DEAD 27
#define SMSCL_OFS_L 33
#define SMSCL_OFS_H 34
#define SMSCL_MODE 35
#define SMSCL_MAX_CURRENT_L 36
#define SMSCL_MAX_CURRENT_H 37

// -------SRAM (Read/Write)--------
#define SMSCL_TORQUE_ENABLE 40
#define SMSCL_ACC 41
#define SMSCL_GOAL_POSITION_L 42
#define SMSCL_GOAL_POSITION_H 43
#define SMSCL_GOAL_TIME_L 44
#define SMSCL_GOAL_TIME_H 45
#define SMSCL_GOAL_SPEED_L 46
#define SMSCL_GOAL_SPEED_H 47
#define SMSCL_LOCK 48

// -------SRAM (Read-only)--------
#define SMSCL_PRESENT_POSITION_L 56
#define SMSCL_PRESENT_POSITION_H 57
#define SMSCL_PRESENT_SPEED_L 58
#define SMSCL_PRESENT_SPEED_H 59
#define SMSCL_PRESENT_LOAD_L 60
#define SMSCL_PRESENT_LOAD_H 61
#define SMSCL_PRESENT_VOLTAGE 62
#define SMSCL_PRESENT_TEMPERATURE 63
#define SMSCL_REGISTERED_INSTRUCTION 64
#define SMSCL_MOVING 66
#define SMSCL_PRESENT_CURRENT_L 69
#define SMSCL_PRESENT_CURRENT_H 70

#include "SCSerial.h"

class SMSCL : public SCSerial
{
public:
  SMSCL();
  SMSCL(u8 End);
  SMSCL(u8 End, u8 Level);
  virtual int writePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC = 0);                   // Standard write position command for a single servo
  virtual int regWritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC = 0);                // Asynchronous write position command for a single servo (effective with RegWriteAction)
  virtual void syncWritePosEx(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[]);  // Synchronous write position command for multiple servos
  virtual int wheelMode(u8 ID);                                                         // Wheel (continuous rotation) mode
  virtual int writeSpeed(u8 ID, s16 Speed, u8 ACC = 0);                                 // Speed control command in wheel mode
  virtual int enableTorque(u8 ID, u8 Enable);                                           // Torque control command
  virtual int unlockEprom(u8 ID);                                                       // Unlock EEPROM
  virtual int lockEprom(u8 ID);                                                         // Lock EEPROM
  virtual int calibrationOfs(u8 ID);                                                    // Center offset calibration
  virtual int feedback(int ID);                                                         // Get feedback from servo
  virtual int readPos(int ID);                                                          // Read position
  virtual int readSpeed(int ID);                                                        // Read speed
  virtual int readLoad(int ID);                                                         // Read load (percentage of voltage output to motor, 0–1000)
  virtual int readVoltage(int ID);                                                      // Read voltage
  virtual int readTemperature(int ID);                                                  // Read temperature
  virtual int readMove(int ID);                                                         // Read movement status
  virtual int readCurrent(int ID);                                                      // Read current
private:
  u8 Mem[SMSCL_PRESENT_CURRENT_H - SMSCL_PRESENT_POSITION_L + 1];
};

#endif