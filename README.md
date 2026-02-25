# 402 project 2
imu position visualization using quaternions
*cube_quat.cpp* is custom, done before it was added to manual. 
## requirements
* **Hardware:** rpi 4B.
* **Software:** C++.
## what it does
* imuread just reads imu values and produces quaternion using madgwick filter and etc.
* cube_quat is visualisation of cube, orientation of which is controlled by "wasd" or by imuread (parallel threading)


## how to run

1. **clone the repo**
2. **compile imuread**:
```bash
g++ imuread.cpp I2CDevice.cpp -o imuread -lrt
```
4. **compile cube_quat**:
```bash
g++ -std=c++17 cube_quat.cpp -lglut -lGL -lGLU -lX11 -pthread -o cube_quat
```
6. **run together**:
```bash
sudo ./imuread | ./cube_quat
```
