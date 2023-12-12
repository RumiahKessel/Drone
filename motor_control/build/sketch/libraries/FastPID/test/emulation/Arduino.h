#line 1 "C:\\Users\\rukes\\Documents\\UCB\\FA23\\CS149\\Drone\\motor_control\\libraries\\FastPID\\test\\emulation\\Arduino.h"
#include <iostream>
#include <stdint.h>
using namespace std;
static uint32_t __timer = 0; 
static uint32_t __t() {
  __timer += 1000;
  return __timer;
}
#define millis() __t()

#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
