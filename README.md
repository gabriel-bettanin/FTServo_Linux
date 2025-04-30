Original repository found in: https://gitee.com/ftservo/FTServo_Linux

# FTServo_Linux
FEETECH BUS Servo Linux library

# Library compilation
```
mkdir -p build
cd build
cmake ..
cmake --build .
```

# Running examples
```
cd build/examples/SMS_STS/SyncRead
sudo ./WritePos /dev/ttyACM0
```
