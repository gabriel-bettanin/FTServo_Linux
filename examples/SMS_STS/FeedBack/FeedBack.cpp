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
  std::cout << "serial:" << argv[1] << std::endl;
  if (!servo_bus.begin(1000000, argv[1]))
  {
    std::cout << "Failed to init sms/sts motor!" << std::endl;
    return 0;
  }
  while (1)
  {
    int position;
    int speed;
    int load;
    int voltage;
    int temperature;
    int move;
    int current;

    if (servo_bus.feedback(1) != -1)
    {
      position = servo_bus.readPos(-1);
      speed = servo_bus.readSpeed(-1);
      load = servo_bus.readLoad(-1);
      voltage = servo_bus.readVoltage(-1);
      temperature = servo_bus.readTemperature(-1);
      move = servo_bus.readMove(-1);
      current = servo_bus.readCurrent(-1);

      std::cout << "position = " << position << " ";
      std::cout << "speed = " << speed << " ";
      std::cout << "load = " << load << " ";
      std::cout << "voltage = " << voltage << " ";
      std::cout << "temperature = " << temperature << " ";
      std::cout << "move = " << move << " ";
      std::cout << "current = " << current << std::endl;
      usleep(1 * 1000);
    }
    else
    {
      std::cout << "read err" << std::endl;
      sleep(1);
    }

    /* // Position
    position = servo_bus.ReadPos(1);
    if (position != -1)
    {
      std::cout << "position = " << position << std::endl;
      usleep(1 * 1000);
    }
    else
    {
      std::cout << "read position err" << std::endl;
      sleep(1);
    }

    // Voltage
    voltage = servo_bus.ReadVoltage(1);
    if (voltage != -1)
    {
      std::cout << "voltage = " << voltage << std::endl;
      usleep(1 * 1000);
    }
    else
    {
      std::cout << "read voltage err" << std::endl;
      sleep(1);
    }

    // Temperature
    temperature = servo_bus.ReadTemper(1);
    if (temperature != -1)
    {
      std::cout << "temperature = " << temperature << std::endl;
      usleep(1 * 1000);
    }
    else
    {
      std::cout << "read temperature err" << std::endl;
      sleep(1);
    }

    // Speed
    speed = servo_bus.ReadSpeed(1);
    if (position != -1)
    {
      std::cout << "speed = " << position << std::endl;
      usleep(1 * 1000);
    }
    else
    {
      std::cout << "read speed err" << std::endl;
      sleep(1);
    }

    // Load
    load = servo_bus.ReadLoad(1);
    if (load != -1)
    {
      std::cout << "load = " << load << std::endl;
      usleep(1 * 1000);
    }
    else
    {
      std::cout << "read Load err" << std::endl;
      sleep(1);
    }

    // Current
    current = servo_bus.ReadCurrent(1);
    if (current != -1)
    {
      std::cout << "current = " << current << std::endl;
      usleep(1 * 1000);
    }
    else
    {
      std::cout << "read Current err" << std::endl;
      sleep(1);
    }

    // Move
    move = servo_bus.ReadMove(1);
    if (move != -1)
    {
      std::cout << "Move = " << move << std::endl;
      usleep(1 * 1000);
    }
    else
    {
      std::cout << "read Move err" << std::endl;
      sleep(1);
    }*/
  }

  servo_bus.end();
  return 1;
}
