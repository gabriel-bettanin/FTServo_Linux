/* 
Ping command test, test whether the corresponding ID servo on the bus is ready, the broadcast command only applies to only one servo on the bus.
*/

#include <iostream>
#include "SCServo.h"

SMS_STS servo_bus;

int main(int argc, char ** argv)
{
  if (argc < 2)
  {
    std::cout << "argc error!" << std::endl;
    return 0;
  }

  std::cout << "Serial port:" << argv[1] << std::endl;
  if (!servo_bus.begin(1000000, argv[1]))
  {
    std::cout << "Failed to init sms/sts motor!" << std::endl;
    return 0;
  }

  std::cout << "Pinging IDs..." << std::endl;
  for (int i = 1; i < 10; i++)
  {
    if (servo_bus.Ping(i) != -1)
    {
      std::cout << "Found ID: " << i << std::endl;
    }
  }

  servo_bus.end();
  return 1;
}
