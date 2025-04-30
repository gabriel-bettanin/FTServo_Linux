/*
 * SCS.h
 * Feit Serial Servo Communication Layer Protocol program
 * Date: 2022.3.29
 * Author.
 */

#ifndef _SCS_H
#define _SCS_H

#include "INST.h"

class SCS
{
public:
  SCS();
  SCS(u8 End);
  SCS(u8 End, u8 Level);
  int genWrite(u8 ID, u8 MemAddr, u8 * nDat, u8 nLen);              // Normal Write Command
  int regWrite(u8 ID, u8 MemAddr, u8 * nDat, u8 nLen);              // Asynchronous write command
  int regWriteAction(u8 ID = 0xfe);                                 // Asynchronous write execution instructions
  void snycWrite(u8 ID[], u8 IDN, u8 MemAddr, u8 * nDat, u8 nLen);  // Synchronous write command
  int writeByte(u8 ID, u8 MemAddr, u8 bDat);                        // Write 1 byte
  int writeWord(u8 ID, u8 MemAddr, u16 wDat);                       // Write 2 bytes
  int read(u8 ID, u8 MemAddr, u8 * nData, u8 nLen);                 // Read command
  int readByte(u8 ID, u8 MemAddr);                                  // Read 1 byte
  int readWord(u8 ID, u8 MemAddr);                                  // Read 2 bytes
  int ping(u8 ID);                                                  // Ping
  int syncReadPacketTx(u8 ID[], u8 IDN, u8 MemAddr, u8 nLen);       // Synchronized Read Command Packet Send
  int syncReadPacketRx(u8 ID, u8 * nDat);                           // Synchronized Read return packet decode, success return memory bytes, failure return 0
  int syncReadRxPacketToByte();                                     // Decode a byte
  int syncReadRxPacketToWrod(u8 negBit = 0);                        // Decode two bytes, negBit for direction as, negBit=0 means no direction
  void syncReadBegin(u8 IDN, u8 rxLen);                             // Synchronized reading started
  void syncReadEnd();                                               // Synchronized reading end
  u8 Level;                                                         // Servo return level
  u8 End;                                                           // Processor size end structure
  u8 Error;                                                         // Servo Status
  u8 syncReadRxPacketIndex;
  u8 syncReadRxPacketLen;
  u8 * syncReadRxPacket;
  u8 * syncReadRxBuff;
  u16 syncReadRxBuffLen;
  u16 syncReadRxBuffMax;

protected:
  virtual int writeSCS(unsigned char * nDat, int nLen) = 0;
  virtual int readSCS(unsigned char * nDat, int nLen) = 0;
  virtual int writeSCS(unsigned char bDat) = 0;
  virtual void rFlushSCS() = 0;
  virtual void wFlushSCS() = 0;

protected:
  void writeBuf(u8 ID, u8 MemAddr, u8 * nDat, u8 nLen, u8 Fun);
  void host2SCS(u8 * DataL, u8 * DataH, u16 Data);  // 1 16-bit number split into 2 8-bit numbers
  u16 SCS2Host(u8 DataL, u8 DataH);                 // 2 8-digit numbers combined into 1 16-digit number
  int ack(u8 ID);                                   // Return answer
};

  #endif