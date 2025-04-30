/*
 * SMSCL.cpp
 * ����SMSCLϵ�д��ж���ӿ�
 * ����: 2020.6.17
 * ����: 
 */

#include "SMSCL.h"

SMSCL::SMSCL() { End = 0; }

SMSCL::SMSCL(u8 End) : SCSerial(End) {}

SMSCL::SMSCL(u8 End, u8 Level) : SCSerial(End, Level) {}

int SMSCL::writePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC)
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

  return genWrite(ID, SMSCL_ACC, bBuf, 7);
}

int SMSCL::regWritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC)
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

  return regWrite(ID, SMSCL_ACC, bBuf, 7);
}

void SMSCL::syncWritePosEx(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[])
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
  snycWrite(ID, IDN, SMSCL_ACC, (u8 *)offbuf, 7);
}

int SMSCL::wheelMode(u8 ID) { return writeByte(ID, SMSCL_MODE, 1); }

int SMSCL::writeSpeed(u8 ID, s16 Speed, u8 ACC)
{
  if (Speed < 0)
  {
    Speed = -Speed;
    Speed |= (1 << 15);
  }
  u8 bBuf[2];
  bBuf[0] = ACC;
  genWrite(ID, SMSCL_ACC, bBuf, 1);
  host2SCS(bBuf + 0, bBuf + 1, Speed);

  return genWrite(ID, SMSCL_GOAL_SPEED_L, bBuf, 2);
}

int SMSCL::enableTorque(u8 ID, u8 Enable) { return writeByte(ID, SMSCL_TORQUE_ENABLE, Enable); }

int SMSCL::unlockEprom(u8 ID) { return writeByte(ID, SMSCL_LOCK, 0); }

int SMSCL::lockEprom(u8 ID) { return writeByte(ID, SMSCL_LOCK, 1); }

int SMSCL::calibrationOfs(u8 ID) { return writeByte(ID, SMSCL_TORQUE_ENABLE, 128); }

int SMSCL::feedback(int ID)
{
  int nLen = read(ID, SMSCL_PRESENT_POSITION_L, Mem, sizeof(Mem));
  if (nLen != sizeof(Mem))
  {
    Err = 1;
    return -1;
  }
  Err = 0;
  return nLen;
}

int SMSCL::readPos(int ID)
{
  int Pos = -1;
  if (ID == -1)
  {
    Pos = Mem[SMSCL_PRESENT_POSITION_H - SMSCL_PRESENT_POSITION_L];
    Pos <<= 8;
    Pos |= Mem[SMSCL_PRESENT_POSITION_L - SMSCL_PRESENT_POSITION_L];
  }
  else
  {
    Err = 0;
    Pos = readWord(ID, SMSCL_PRESENT_POSITION_L);
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

int SMSCL::readSpeed(int ID)
{
  int Speed = -1;
  if (ID == -1)
  {
    Speed = Mem[SMSCL_PRESENT_SPEED_H - SMSCL_PRESENT_POSITION_L];
    Speed <<= 8;
    Speed |= Mem[SMSCL_PRESENT_SPEED_L - SMSCL_PRESENT_POSITION_L];
  }
  else
  {
    Err = 0;
    Speed = readWord(ID, SMSCL_PRESENT_SPEED_L);
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

int SMSCL::readLoad(int ID)
{
  int Load = -1;
  if (ID == -1)
  {
    Load = Mem[SMSCL_PRESENT_LOAD_H - SMSCL_PRESENT_POSITION_L];
    Load <<= 8;
    Load |= Mem[SMSCL_PRESENT_LOAD_L - SMSCL_PRESENT_POSITION_L];
  }
  else
  {
    Err = 0;
    Load = readWord(ID, SMSCL_PRESENT_LOAD_L);
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

int SMSCL::readVoltage(int ID)
{
  int Voltage = -1;
  if (ID == -1)
  {
    Voltage = Mem[SMSCL_PRESENT_VOLTAGE - SMSCL_PRESENT_POSITION_L];
  }
  else
  {
    Err = 0;
    Voltage = readByte(ID, SMSCL_PRESENT_VOLTAGE);
    if (Voltage == -1)
    {
      Err = 1;
    }
  }
  return Voltage;
}

int SMSCL::readTemperature(int ID)
{
  int Temper = -1;
  if (ID == -1)
  {
    Temper = Mem[SMSCL_PRESENT_TEMPERATURE - SMSCL_PRESENT_POSITION_L];
  }
  else
  {
    Err = 0;
    Temper = readByte(ID, SMSCL_PRESENT_TEMPERATURE);
    if (Temper == -1)
    {
      Err = 1;
    }
  }
  return Temper;
}

int SMSCL::readMove(int ID)
{
  int Move = -1;
  if (ID == -1)
  {
    Move = Mem[SMSCL_MOVING - SMSCL_PRESENT_POSITION_L];
  }
  else
  {
    Err = 0;
    Move = readByte(ID, SMSCL_MOVING);
    if (Move == -1)
    {
      Err = 1;
    }
  }
  return Move;
}

int SMSCL::readCurrent(int ID)
{
  int Current = -1;
  if (ID == -1)
  {
    Current = Mem[SMSCL_PRESENT_CURRENT_H - SMSCL_PRESENT_POSITION_L];
    Current <<= 8;
    Current |= Mem[SMSCL_PRESENT_CURRENT_L - SMSCL_PRESENT_POSITION_L];
  }
  else
  {
    Err = 0;
    Current = readWord(ID, SMSCL_PRESENT_CURRENT_L);
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
