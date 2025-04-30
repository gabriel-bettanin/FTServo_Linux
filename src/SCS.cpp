/*
 * SCS.cpp
 * Feit Serial Servo Communication Layer Protocol program
 * Date: 2022.3.29
 * Author. 
 */
#include "SCS.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

SCS::SCS()
{
  Level = 1;  //All commands except broadcast commands return an answer
  Error = 0;
}

SCS::SCS(u8 End)
{
  Level = 1;
  this->End = End;
  Error = 0;
}

SCS::SCS(u8 End, u8 Level)
{
  this->Level = Level;
  this->End = End;
  Error = 0;
}

// 1 16-bit number split into 2 8-bit numbers
// DataL is low, DataH is high
void SCS::host2SCS(u8 * DataL, u8 * DataH, u16 Data)
{
  if (End)
  {
    *DataL = (Data >> 8);
    *DataH = (Data & 0xff);
  }
  else
  {
    *DataH = (Data >> 8);
    *DataL = (Data & 0xff);
  }
}

// 2 8-bit numbers combined into 1 16-bit number
// DataL is the low bit, DataH is the high bit
u16 SCS::SCS2Host(u8 DataL, u8 DataH)
{
  u16 Data;
  if (End)
  {
    Data = DataL;
    Data <<= 8;
    Data |= DataH;
  }
  else
  {
    Data = DataH;
    Data <<= 8;
    Data |= DataL;
  }
  return Data;
}

void SCS::writeBuf(u8 ID, u8 MemAddr, u8 * nDat, u8 nLen, u8 Fun)
{
  u8 msgLen = 2;
  u8 bBuf[6];
  u8 CheckSum = 0;
  bBuf[0] = 0xff;
  bBuf[1] = 0xff;
  bBuf[2] = ID;
  bBuf[4] = Fun;
  if (nDat)
  {
    msgLen += nLen + 1;
    bBuf[3] = msgLen;
    bBuf[5] = MemAddr;
    writeSCS(bBuf, 6);
  }
  else
  {
    bBuf[3] = msgLen;
    writeSCS(bBuf, 5);
  }
  CheckSum = ID + msgLen + Fun + MemAddr;
  u8 i = 0;
  if (nDat)
  {
    for (i = 0; i < nLen; i++)
    {
      CheckSum += nDat[i];
    }
    writeSCS(nDat, nLen);
  }
  writeSCS(~CheckSum);
}

// Normal write instruction
// Servo ID, MemAddr memory table address, write data, write length
int SCS::genWrite(u8 ID, u8 MemAddr, u8 * nDat, u8 nLen)
{
  rFlushSCS();
  writeBuf(ID, MemAddr, nDat, nLen, INST_WRITE);
  wFlushSCS();
  return ack(ID);
}

// Asynchronous write instruction
// Servo ID, MemAddr memory table address, write data, write length
int SCS::regWrite(u8 ID, u8 MemAddr, u8 * nDat, u8 nLen)
{
  rFlushSCS();
  writeBuf(ID, MemAddr, nDat, nLen, INST_REG_WRITE);
  wFlushSCS();
  return ack(ID);
}

// Asynchronous Write Execution Instruction
// Servo ID
int SCS::regWriteAction(u8 ID)
{
  rFlushSCS();
  writeBuf(ID, 0, NULL, 0, INST_REG_ACTION);
  wFlushSCS();
  return ack(ID);
}

// Synchronized write instruction
// Servo ID[] array, IDN array length, MemAddr memory table address, write data, write length
void SCS::snycWrite(u8 ID[], u8 IDN, u8 MemAddr, u8 * nDat, u8 nLen)
{
  rFlushSCS();
  u8 mesLen = ((nLen + 1) * IDN + 4);
  u8 Sum = 0;
  u8 bBuf[7];
  bBuf[0] = 0xff;
  bBuf[1] = 0xff;
  bBuf[2] = 0xfe;
  bBuf[3] = mesLen;
  bBuf[4] = INST_SYNC_WRITE;
  bBuf[5] = MemAddr;
  bBuf[6] = nLen;
  writeSCS(bBuf, 7);

  Sum = 0xfe + mesLen + INST_SYNC_WRITE + MemAddr + nLen;
  u8 i, j;
  for (i = 0; i < IDN; i++)
  {
    writeSCS(ID[i]);
    writeSCS(nDat + i * nLen, nLen);
    Sum += ID[i];
    for (j = 0; j < nLen; j++)
    {
      Sum += nDat[i * nLen + j];
    }
  }
  writeSCS(~Sum);
  wFlushSCS();
}

// Write 1 byte
int SCS::writeByte(u8 ID, u8 MemAddr, u8 bDat)
{
  rFlushSCS();
  writeBuf(ID, MemAddr, &bDat, 1, INST_WRITE);
  wFlushSCS();
  return ack(ID);
}

// Write 2 bytes
int SCS::writeWord(u8 ID, u8 MemAddr, u16 wDat)
{
  u8 bBuf[2];
  host2SCS(bBuf + 0, bBuf + 1, wDat);
  rFlushSCS();
  writeBuf(ID, MemAddr, bBuf, 2, INST_WRITE);
  wFlushSCS();
  return ack(ID);
}

// Read instruction
// Servo ID, MemAddr memory table address, return data nData, data length nLen
int SCS::read(u8 ID, u8 MemAddr, u8 * nData, u8 nLen)
{
  rFlushSCS();
  writeBuf(ID, MemAddr, &nLen, 1, INST_READ);
  wFlushSCS();

  u8 bBuf[255];
  u8 i;
  u8 calSum = 0;
  int Size = readSCS(bBuf, nLen + 6);
  //printf("nLen+6 = %d, Size = %d\n", nLen+6, Size);
  if (Size != (nLen + 6))
  {
    return 0;
  }
  //for(i=0; i<Size; i++){
  //printf("%x\n", bBuf[i]);
  //}
  if (bBuf[0] != 0xff || bBuf[1] != 0xff)
  {
    return 0;
  }
  for (i = 2; i < (Size - 1); i++)
  {
    calSum += bBuf[i];
  }
  calSum = ~calSum;
  if (calSum != bBuf[Size - 1])
  {
    return 0;
  }
  memcpy(nData, bBuf + 5, nLen);
  Error = bBuf[4];
  return nLen;
}

//read 1 byte, timeout returns -1
int SCS::readByte(u8 ID, u8 MemAddr)
{
  u8 bDat;
  int Size = read(ID, MemAddr, &bDat, 1);
  if (Size != 1)
  {
    return -1;
  }
  else
  {
    return bDat;
  }
}

//read 2 bytes, timeout returns -1
int SCS::readWord(u8 ID, u8 MemAddr)
{
  u8 nDat[2];
  int Size;
  u16 wDat;
  Size = read(ID, MemAddr, nDat, 2);
  if (Size != 2) return -1;
  wDat = SCS2Host(nDat[0], nDat[1]);
  return wDat;
}

//Ping command, return servo ID, timeout return -1
int SCS::ping(u8 ID)
{
  rFlushSCS();
  writeBuf(ID, 0, NULL, 0, INST_PING);
  wFlushSCS();
  Error = 0;

  u8 bBuf[6];
  u8 i;
  u8 calSum = 0;
  int Size = readSCS(bBuf, 6);
  if (Size != 6)
  {
    return -1;
  }
  if (bBuf[0] != 0xff || bBuf[1] != 0xff)
  {
    return -1;
  }
  if (bBuf[2] != ID && ID != 0xfe)
  {
    return -1;
  }
  if (bBuf[3] != 2)
  {
    return -1;
  }
  for (i = 2; i < (Size - 1); i++)
  {
    calSum += bBuf[i];
  }
  calSum = ~calSum;
  if (calSum != bBuf[Size - 1])
  {
    return -1;
  }
  Error = bBuf[2];
  return Error;
}

// Return answer
int SCS::ack(u8 ID)
{
  Error = 0;
  if (ID != 0xfe && Level)
  {
    u8 bBuf[6];
    u8 i;
    u8 calSum = 0;
    int Size = readSCS(bBuf, 6);
    if (Size != 6)
    {
      return 0;
    }
    if (bBuf[0] != 0xff || bBuf[1] != 0xff)
    {
      return 0;
    }
    if (bBuf[2] != ID)
    {
      return 0;
    }
    if (bBuf[3] != 2)
    {
      return 0;
    }
    for (i = 2; i < (Size - 1); i++)
    {
      calSum += bBuf[i];
    }
    calSum = ~calSum;
    if (calSum != bBuf[Size - 1])
    {
      return 0;
    }
    Error = bBuf[4];
  }
  return 1;
}

// Synchronized Read Command Packet Send
int SCS::syncReadPacketTx(u8 ID[], u8 IDN, u8 MemAddr, u8 nLen)
{
  rFlushSCS();
  syncReadRxPacketLen = nLen;
  u8 checkSum = (4 + 0xfe) + IDN + MemAddr + nLen + INST_SYNC_READ;
  u8 i;
  writeSCS(0xff);
  writeSCS(0xff);
  writeSCS(0xfe);
  writeSCS(IDN + 4);
  writeSCS(INST_SYNC_READ);
  writeSCS(MemAddr);
  writeSCS(nLen);
  for (i = 0; i < IDN; i++)
  {
    writeSCS(ID[i]);
    checkSum += ID[i];
  }
  checkSum = ~checkSum;
  writeSCS(checkSum);
  wFlushSCS();

  syncReadRxBuffLen = readSCS(syncReadRxBuff, syncReadRxBuffMax);
  return syncReadRxBuffLen;
}

// Synchronized reading started
void SCS::syncReadBegin(u8 IDN, u8 rxLen)
{
  syncReadRxBuffMax = IDN * (rxLen + 6);
  syncReadRxBuff = new u8[syncReadRxBuffMax];
}

// Synchronized reading end
void SCS::syncReadEnd()
{
  if (syncReadRxBuff)
  {
    delete syncReadRxBuff;
    syncReadRxBuff = NULL;
  }
}

// Synchronized Read return packet decode, success return memory bytes, failure return 0
int SCS::syncReadPacketRx(u8 ID, u8 * nDat)
{
  u16 syncReadRxBuffIndex = 0;
  syncReadRxPacket = nDat;
  syncReadRxPacketIndex = 0;
  while ((syncReadRxBuffIndex + 6 + syncReadRxPacketLen) <= syncReadRxBuffLen)
  {
    u8 bBuf[] = {0, 0, 0};
    u8 calSum = 0;
    while (syncReadRxBuffIndex < syncReadRxBuffLen)
    {
      bBuf[0] = bBuf[1];
      bBuf[1] = bBuf[2];
      bBuf[2] = syncReadRxBuff[syncReadRxBuffIndex++];
      if (bBuf[0] == 0xff && bBuf[1] == 0xff && bBuf[2] != 0xff)
      {
        break;
      }
    }
    if (bBuf[2] != ID)
    {
      continue;
    }
    if (syncReadRxBuff[syncReadRxBuffIndex++] != (syncReadRxPacketLen + 2))
    {
      continue;
    }
    Error = syncReadRxBuff[syncReadRxBuffIndex++];
    calSum = ID + (syncReadRxPacketLen + 2) + Error;
    for (u8 i = 0; i < syncReadRxPacketLen; i++)
    {
      syncReadRxPacket[i] = syncReadRxBuff[syncReadRxBuffIndex++];
      calSum += syncReadRxPacket[i];
    }
    calSum = ~calSum;
    if (calSum != syncReadRxBuff[syncReadRxBuffIndex++])
    {
      return 0;
    }
    return syncReadRxPacketLen;
  }
  return 0;
}

// Decode a byte
int SCS::syncReadRxPacketToByte()
{
  if (syncReadRxPacketIndex >= syncReadRxPacketLen)
  {
    return -1;
  }
  return syncReadRxPacket[syncReadRxPacketIndex++];
}

// Decode two bytes, negBit for direction as, negBit=0 means no direction
int SCS::syncReadRxPacketToWrod(u8 negBit)
{
  if ((syncReadRxPacketIndex + 1) >= syncReadRxPacketLen)
  {
    return -1;
  }
  int Word = SCS2Host(syncReadRxPacket[syncReadRxPacketIndex], syncReadRxPacket[syncReadRxPacketIndex + 1]);
  syncReadRxPacketIndex += 2;
  if (negBit)
  {
    if (Word & (1 << negBit))
    {
      Word = -(Word & ~(1 << negBit));
    }
  }
  return Word;
}