/* 
Synchronized read command to read back the position and velocity information of both servos ID1 and ID2
*/

#include <iostream>
#include "SCServo.h"

SMS_STS servo_bus;
uint8_t ids[] = {1, 2, 3};
uint8_t rx_packet[4];
int16_t position, speed;

int main(int argc, char ** argv)
{
  if (argc < 2)
  {
    std::cout << "argc error!" << std::endl;
    return 0;
  }
  std::cout << "serial:" << argv[1] << std::endl;

  if (!servo_bus.begin(1000000, argv[1]))
  {
    std::cout << "Failed to init sms/sts motor!" << std::endl;
    return 0;
  }
  servo_bus.syncReadBegin(sizeof(ids), sizeof(rx_packet));

  while (1)
  {
    servo_bus.syncReadPacketTx(ids, sizeof(ids), SMS_STS_PRESENT_POSITION_L, sizeof(rx_packet));  //同步读指令包发送
    for (uint8_t i = 0; i < sizeof(ids); i++)
    {
      // Receive ID[i] synchronized read return packet
      if (!servo_bus.syncReadPacketRx(ids[i], rx_packet))
      {
        std::cout << "ID:" << (int)ids[i] << " sync read error!" << std::endl;
        continue;  // Receive decoding failure
      }
      position = servo_bus.syncReadRxPacketToWrod(15);  // Decode two bytes bit15 is direction bit, parameter = 0 means no direction bit.
      speed = servo_bus.syncReadRxPacketToWrod(15);     // Decode two bytes bit15 is direction bit, parameter = 0 means no direction bit.
      std::cout << "ID:" << int(ids[i]) << " Position:" << position << " Speed:" << speed << std::endl;
    }
    usleep(1 * 1000);
  }

  servo_bus.syncReadEnd();
  servo_bus.end();
  return 1;
}