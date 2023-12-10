#include <Arduino.h>
#include "LSM303D.h"

enum REG_ADDR {
  /*---ACCELERATOR---*/
  OUT_X_A     = 0x28,
  OUT_Y_A     = 0x2A,
  OUT_Z_A     = 0x2C,
  //
  CTRL0       = 0x1F,
  CTRL1       = 0x20,
  CTRL2       = 0x21,
  CTRL3       = 0x22,
  CTRL4       = 0x23,
  CTRL5       = 0x24,
  CTRL6       = 0x25,
  CTRL7       = 0x26
}

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}

void LSM303D::readAcc(void) {
  char data = addr_reg;
  
}