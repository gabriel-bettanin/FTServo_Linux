/*
 * SCSCL.cpp
 * Feit SCSCL series serial servo application layer program
 * Date: 2020.6.17
 * Author.
 */
#include "SCSCL.h"

SCSCL::SCSCL() { End = 1; }

SCSCL::SCSCL(u8 End) : SCSerial(End) {}

SCSCL::SCSCL(u8 End, u8 Level) : SCSerial(End, Level) {}

// Write position command for a single servo
int SCSCL::writePos(u8 ID, u16 Position, u16 Time, u16 Speed)
{
  u8 bBuf[6];
  host2SCS(bBuf + 0, bBuf + 1, Position);
  host2SCS(bBuf + 2, bBuf + 3, Time);
  host2SCS(bBuf + 4, bBuf + 5, Speed);

  return genWrite(ID, SCSCL_GOAL_POSITION_L, bBuf, 6);
}

// Asynchronous write position command for a single servo (takes effect with RegWriteAction)
int SCSCL::regWritePos(u8 ID, u16 Position, u16 Time, u16 Speed)
{
  u8 bBuf[6];
  host2SCS(bBuf + 0, bBuf + 1, Position);
  host2SCS(bBuf + 2, bBuf + 3, Time);
  host2SCS(bBuf + 4, bBuf + 5, Speed);

  return regWrite(ID, SCSCL_GOAL_POSITION_L, bBuf, 6);
}

// Synchronous write position command for multiple servos
void SCSCL::syncWritePos(u8 ID[], u8 IDN, u16 Position[], u16 Time[], u16 Speed[])
{
  u8 offbuf[IDN][6];
  for (u8 i = 0; i < IDN; i++)
  {
    u8 bBuf[6];
    u16 T, V;
    if (Time)
    {
      T = Time[i];
    }
    else
    {
      T = 0;
    }
    if (Speed)
    {
      V = Speed[i];
    }
    else
    {
      V = 0;
    }
    host2SCS(bBuf + 0, bBuf + 1, Position[i]);
    host2SCS(bBuf + 2, bBuf + 3, T);
    host2SCS(bBuf + 4, bBuf + 5, V);
    memcpy(offbuf[i], bBuf, 6);
  }
  snycWrite(ID, IDN, SCSCL_GOAL_POSITION_L, (u8 *)offbuf, 6);
}

// Set PWM output mode
int SCSCL::pwmMode(u8 ID)
{
  u8 bBuf[4];
  bBuf[0] = 0;
  bBuf[1] = 0;
  bBuf[2] = 0;
  bBuf[3] = 0;
  return genWrite(ID, SCSCL_MIN_ANGLE_LIMIT_L, bBuf, 4);
}

// Command for PWM output mode
int SCSCL::writePWM(u8 ID, s16 pwmOut)
{
  if (pwmOut < 0)
  {
    pwmOut = -pwmOut;
    pwmOut |= (1 << 10);
  }
  u8 bBuf[2];
  host2SCS(bBuf + 0, bBuf + 1, pwmOut);

  return genWrite(ID, SCSCL_GOAL_TIME_L, bBuf, 2);
}

// Torque control command
int SCSCL::enableTorque(u8 ID, u8 Enable) { return writeByte(ID, SCSCL_TORQUE_ENABLE, Enable); }

// Unlock EEPROM
int SCSCL::unlockEprom(u8 ID) { return writeByte(ID, SCSCL_LOCK, 0); }

// Lock EEPROM
int SCSCL::lockEprom(u8 ID) { return writeByte(ID, SCSCL_LOCK, 1); }

// Get feedback from servo
int SCSCL::feedback(int ID)
{
  int nLen = read(ID, SCSCL_PRESENT_POSITION_L, Mem, sizeof(Mem));
  if (nLen != sizeof(Mem))
  {
    Err = 1;
    return -1;
  }
  Err = 0;
  return nLen;
}

// Read position
int SCSCL::readPos(int ID)
{
  int Pos = -1;
  if (ID == -1)
  {
    Pos = Mem[SCSCL_PRESENT_POSITION_L - SCSCL_PRESENT_POSITION_L];
    Pos <<= 8;
    Pos |= Mem[SCSCL_PRESENT_POSITION_H - SCSCL_PRESENT_POSITION_L];
  }
  else
  {
    Err = 0;
    Pos = readWord(ID, SCSCL_PRESENT_POSITION_L);
    if (Pos == -1)
    {
      Err = 1;
    }
  }
  return Pos;
}

// Read speed
int SCSCL::readSpeed(int ID)
{
  int Speed = -1;
  if (ID == -1)
  {
    Speed = Mem[SCSCL_PRESENT_SPEED_L - SCSCL_PRESENT_POSITION_L];
    Speed <<= 8;
    Speed |= Mem[SCSCL_PRESENT_SPEED_H - SCSCL_PRESENT_POSITION_L];
  }
  else
  {
    Err = 0;
    Speed = readWord(ID, SCSCL_PRESENT_SPEED_L);
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

// Read load (percentage of voltage output to motor, 0~1000)
int SCSCL::readLoad(int ID)
{
  int Load = -1;
  if (ID == -1)
  {
    Load = Mem[SCSCL_PRESENT_LOAD_L - SCSCL_PRESENT_POSITION_L];
    Load <<= 8;
    Load |= Mem[SCSCL_PRESENT_LOAD_H - SCSCL_PRESENT_POSITION_L];
  }
  else
  {
    Err = 0;
    Load = readWord(ID, SCSCL_PRESENT_LOAD_L);
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
int SCSCL::readVoltage(int ID)
{
  int Voltage = -1;
  if (ID == -1)
  {
    Voltage = Mem[SCSCL_PRESENT_VOLTAGE - SCSCL_PRESENT_POSITION_L];
  }
  else
  {
    Err = 0;
    Voltage = readByte(ID, SCSCL_PRESENT_VOLTAGE);
    if (Voltage == -1)
    {
      Err = 1;
    }
  }
  return Voltage;
}

// Read temperature
int SCSCL::readTemperature(int ID)
{
  int Temper = -1;
  if (ID == -1)
  {
    Temper = Mem[SCSCL_PRESENT_TEMPERATURE - SCSCL_PRESENT_POSITION_L];
  }
  else
  {
    Err = 0;
    Temper = readByte(ID, SCSCL_PRESENT_TEMPERATURE);
    if (Temper == -1)
    {
      Err = 1;
    }
  }
  return Temper;
}

// Read movement state
int SCSCL::readMove(int ID)
{
  int Move = -1;
  if (ID == -1)
  {
    Move = Mem[SCSCL_MOVING - SCSCL_PRESENT_POSITION_L];
  }
  else
  {
    Err = 0;
    Move = readByte(ID, SCSCL_MOVING);
    if (Move == -1)
    {
      Err = 1;
    }
  }
  return Move;
}

// Read current
int SCSCL::readCurrent(int ID)
{
  int Current = -1;
  if (ID == -1)
  {
    Current = Mem[SCSCL_PRESENT_CURRENT_L - SCSCL_PRESENT_POSITION_L];
    Current <<= 8;
    Current |= Mem[SCSCL_PRESENT_CURRENT_H - SCSCL_PRESENT_POSITION_L];
  }
  else
  {
    Err = 0;
    Current = readWord(ID, SCSCL_PRESENT_CURRENT_L);
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