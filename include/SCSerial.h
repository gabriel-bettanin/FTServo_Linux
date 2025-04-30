/*
 * SCSerial.h
 * Feit Serial Servo Hardware Interface Layer program
 * Date: 2022.3.29
 * Author.
 */

#ifndef _SCSERIAL_H
#define _SCSERIAL_H

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include "SCS.h"

class SCSerial : public SCS
{
public:
  SCSerial();
  SCSerial(u8 End);
  SCSerial(u8 End, u8 Level);

protected:
  int writeSCS(unsigned char * nDat, int nLen);  // Output nLen bytes
  int readSCS(unsigned char * nDat, int nLen);   // Input nLen bytes
  int writeSCS(unsigned char bDat);              // Output 1 byte
  void rFlushSCS();                              // Flush input buffer
  void wFlushSCS();                              // Flush output buffer

public:
  unsigned long int IOTimeOut;  // Input/output timeout
  int Err;                      // Error code

public:
  virtual int getErr() { return Err; }
  virtual int setBaudRate(int baudRate);
  virtual bool begin(int baudRate, const char * serialPort);
  virtual void end();

protected:
  int fd;                 // Serial port file descriptor
  struct termios orgopt;  // Original serial port options
  struct termios curopt;  // Current serial port options
  unsigned char txBuf[255];
  int txBufLen;
};

#endif