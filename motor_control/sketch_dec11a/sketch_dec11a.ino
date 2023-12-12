#include "CalLib.h"
#include <EEPROM.h>

#define  SERIAL_PORT_SPEED  115200

void setup() {
  // put your setup code here, to run once:
  Serial.begin(SERIAL_PORT_SPEED);
  Serial.println("ArduinoMagCal starting");
  Serial.println("Enter s to save current data to EEPROM");
  Serial.println("Writing to eeprom");
  CALLIB_DATA dat;
//  for (byte i = 0; i < 10; i++)
//    EEPROM.write(0, i);
//  EEPROM.write(0, 0x21);
//  delay(1000);
//  Serial.printf("reading EEPROM: %x", EEPROM.read(0));
   bool status = calLibRead(0, &dat);
   Serial.println(status);
   Serial.printf("%x, %x, %x, %x, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f\n", dat.validL, dat.validH, dat.magValid, dat.pad, dat.magMin[0], dat.magMin[1], dat.magMin[2], dat.magMax[0], dat.magMax[1], dat.magMax[2]);
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
