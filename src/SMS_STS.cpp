/*
 * SMS_STS.cpp
 * Feit SMS/STS series serial servo application layer program
 * Date: 2021.12.8
 * Author.
 */
#include "SMS_STS.h"

SMS_STS::SMS_STS() { End = 0; }

SMS_STS::SMS_STS(u8 End) : SCSerial(End) {}

SMS_STS::SMS_STS(u8 End, u8 Level) : SCSerial(End, Level) {}

// Standard write position command for a single servo
int SMS_STS::writePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC)
{
  if (Position < 0)
  {
    Position = -Position;
    Position |= (1 << 15);
  }
  u8 bBuf[7];
  bBuf[0] = ACC;
  host2SCS(bBuf + 1, bBuf + 2, Position);
  host2SCS(bBuf + 3, bBuf + 4, 0);
  host2SCS(bBuf + 5, bBuf + 6, Speed);

  return genWrite(ID, SMS_STS_ACC, bBuf, 7);
}

// Asynchronous write position command for a single servo (requires RegWriteAction)
int SMS_STS::regWritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC)
{
  if (Position < 0)
  {
    Position = -Position;
    Position |= (1 << 15);
  }
  u8 bBuf[7];
  bBuf[0] = ACC;
  host2SCS(bBuf + 1, bBuf + 2, Position);
  host2SCS(bBuf + 3, bBuf + 4, 0);
  host2SCS(bBuf + 5, bBuf + 6, Speed);

  return regWrite(ID, SMS_STS_ACC, bBuf, 7);
}

// Synchronous write position command for multiple servos
void SMS_STS::syncWritePosEx(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[])
{
  u8 offbuf[IDN][7];
  for (u8 i = 0; i < IDN; i++)
  {
    if (Position[i] < 0)
    {
      Position[i] = -Position[i];
      Position[i] |= (1 << 15);
    }
    u8 bBuf[7];
    u16 V;
    if (Speed)
    {
      V = Speed[i];
    }
    else
    {
      V = 0;
    }
    if (ACC)
    {
      bBuf[0] = ACC[i];
    }
    else
    {
      bBuf[0] = 0;
    }
    host2SCS(bBuf + 1, bBuf + 2, Position[i]);
    host2SCS(bBuf + 3, bBuf + 4, 0);
    host2SCS(bBuf + 5, bBuf + 6, V);
    memcpy(offbuf[i], bBuf, 7);
  }
  snycWrite(ID, IDN, SMS_STS_ACC, (u8 *)offbuf, 7);
}

// Constant speed mode
int SMS_STS::wheelMode(u8 ID) { return writeByte(ID, SMS_STS_MODE, 1); }

// Command to control speed in constant speed mode
int SMS_STS::writeSpeed(u8 ID, s16 Speed, u8 ACC)
{
  if (Speed < 0)
  {
    Speed = -Speed;
    Speed |= (1 << 15);
  }
  u8 bBuf[2];
  bBuf[0] = ACC;
  genWrite(ID, SMS_STS_ACC, bBuf, 1);
  host2SCS(bBuf + 0, bBuf + 1, Speed);

  return genWrite(ID, SMS_STS_GOAL_SPEED_L, bBuf, 2);
}

// Torque control command
int SMS_STS::enableTorque(u8 ID, u8 Enable) { return writeByte(ID, SMS_STS_TORQUE_ENABLE, Enable); }

// Unlock EEPROM
int SMS_STS::unlockEprom(u8 ID) { return writeByte(ID, SMS_STS_LOCK, 0); }

// Lock EEPROM
int SMS_STS::lockEprom(u8 ID) { return writeByte(ID, SMS_STS_LOCK, 1); }

// Center offset calibration
int SMS_STS::calibrationOfs(u8 ID) { return writeByte(ID, SMS_STS_TORQUE_ENABLE, 128); }

// Feedback from servo
int SMS_STS::feedback(int ID)
{
  int nLen = read(ID, SMS_STS_PRESENT_POSITION_L, Mem, sizeof(Mem));
  if (nLen != sizeof(Mem))
  {
    Err = 1;
    return -1;
  }
  Err = 0;
  return nLen;
}

// Read position
int SMS_STS::readPos(int ID)
{
  int Pos = -1;
  if (ID == -1)
  {
    Pos = Mem[SMS_STS_PRESENT_POSITION_H - SMS_STS_PRESENT_POSITION_L];
    Pos <<= 8;
    Pos |= Mem[SMS_STS_PRESENT_POSITION_L - SMS_STS_PRESENT_POSITION_L];
  }
  else
  {
    Err = 0;
    Pos = readWord(ID, SMS_STS_PRESENT_POSITION_L);
    if (Pos == -1)
    {
      Err = 1;
    }
  }
  if (!Err && (Pos & (1 << 15)))
  {
    Pos = -(Pos & ~(1 << 15));
  }

  return Pos;
}

// Read speed
int SMS_STS::readSpeed(int ID)
{
  int Speed = -1;
  if (ID == -1)
  {
    Speed = Mem[SMS_STS_PRESENT_SPEED_H - SMS_STS_PRESENT_POSITION_L];
    Speed <<= 8;
    Speed |= Mem[SMS_STS_PRESENT_SPEED_L - SMS_STS_PRESENT_POSITION_L];
  }
  else
  {
    Err = 0;
    Speed = readWord(ID, SMS_STS_PRESENT_SPEED_L);
    if (Speed == -1)
    {
      Err = 1;
      return -1;
    }
  }
  if (!Err && (Speed & (1 << 15)))
  {
    Speed = -(Speed & ~(1 << 15));
  }
  return Speed;
}

// Read load (percentage of voltage output to motor, 0–1000)
int SMS_STS::readLoad(int ID)
{
  int Load = -1;
  if (ID == -1)
  {
    Load = Mem[SMS_STS_PRESENT_LOAD_H - SMS_STS_PRESENT_POSITION_L];
    Load <<= 8;
    Load |= Mem[SMS_STS_PRESENT_LOAD_L - SMS_STS_PRESENT_POSITION_L];
  }
  else
  {
    Err = 0;
    Load = readWord(ID, SMS_STS_PRESENT_LOAD_L);
    if (Load == -1)
    {
      Err = 1;
    }
  }
  if (!Err && (Load & (1 << 10)))
  {
    Load = -(Load & ~(1 << 10));
  }
  return Load;
}

// Read voltage
int SMS_STS::readVoltage(int ID)
{
  int Voltage = -1;
  if (ID == -1)
  {
    Voltage = Mem[SMS_STS_PRESENT_VOLTAGE - SMS_STS_PRESENT_POSITION_L];
  }
  else
  {
    Err = 0;
    Voltage = readByte(ID, SMS_STS_PRESENT_VOLTAGE);
    if (Voltage == -1)
    {
      Err = 1;
    }
  }
  return Voltage;
}

// Read temperature
int SMS_STS::readTemperature(int ID)
{
  int Temper = -1;
  if (ID == -1)
  {
    Temper = Mem[SMS_STS_PRESENT_TEMPERATURE - SMS_STS_PRESENT_POSITION_L];
  }
  else
  {
    Err = 0;
    Temper = readByte(ID, SMS_STS_PRESENT_TEMPERATURE);
    if (Temper == -1)
    {
      Err = 1;
    }
  }
  return Temper;
}

// Read movement state
int SMS_STS::readMove(int ID)
{
  int Move = -1;
  if (ID == -1)
  {
    Move = Mem[SMS_STS_MOVING - SMS_STS_PRESENT_POSITION_L];
  }
  else
  {
    Err = 0;
    Move = readByte(ID, SMS_STS_MOVING);
    if (Move == -1)
    {
      Err = 1;
    }
  }
  return Move;
}

// Read current
int SMS_STS::readCurrent(int ID)
{
  int Current = -1;
  if (ID == -1)
  {
    Current = Mem[SMS_STS_PRESENT_CURRENT_H - SMS_STS_PRESENT_POSITION_L];
    Current <<= 8;
    Current |= Mem[SMS_STS_PRESENT_CURRENT_L - SMS_STS_PRESENT_POSITION_L];
  }
  else
  {
    Err = 0;
    Current = readWord(ID, SMS_STS_PRESENT_CURRENT_L);
    if (Current == -1)
    {
      Err = 1;
      return -1;
    }
  }
  if (!Err && (Current & (1 << 15)))
  {
    Current = -(Current & ~(1 << 15));
  }
  return Current;
}
