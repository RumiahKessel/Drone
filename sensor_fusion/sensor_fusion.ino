#include <Wire.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "CalLib.h"
#include <EEPROM.h>

#include <LSM303.h>
#include <L3G.h>
#include <LPS.h>
#include <esp_timer.h>
#include <math.h>
#include "ESP32TimerInterrupt.h"

#define GD20HM303D_6a                   // GD20H + M303D at address 0x6a
#define GD20HM303D_6b                   // GD20H + M303D at address 0x6b

#define LPS25H_5c                       // LPS25H at standard address

#define  SERIAL_PORT_SPEED  115200

void setup() {
    calLibRead(0, &calData);

    calData.magValid = false;
    for (int i = 0; i < 3; i++) {
    calData.magMin[i] = 10000000;                    // init mag cal data
    calData.magMax[i] = -10000000;
    }

    Serial.begin(SERIAL_PORT_SPEED);
    Serial.println("ArduinoMagCal starting");
  Serial.println("Enter s to save current data to EEPROM");
  Wire.begin();
   
  imu = RTIMU::createIMU(&settings);                 // create the imu object
  imu->IMUInit();
  imu->setCalibrationMode(true);                     // make sure we get raw data
  Serial.print("ArduinoIMU calibrating device "); Serial.println(imu->IMUName());
}

void loop()
{  
  boolean changed;
  RTVector3 mag;
  
  if (imu->IMURead()) {                                 // get the latest data
    changed = false;
    mag = imu->getCompass();
    for (int i = 0; i < 3; i++) {
      if (mag.data(i) < calData.magMin[i]) {
        calData.magMin[i] = mag.data(i);
        changed = true;
      }
      if (mag.data(i) > calData.magMax[i]) {
        calData.magMax[i] = mag.data(i);
        changed = true;
      }
    }
 
    if (changed) {
      Serial.println("-------");
      Serial.print("minX: "); Serial.print(calData.magMin[0]);
      Serial.print(" maxX: "); Serial.print(calData.magMax[0]); Serial.println();
      Serial.print("minY: "); Serial.print(calData.magMin[1]);
      Serial.print(" maxY: "); Serial.print(calData.magMax[1]); Serial.println();
      Serial.print("minZ: "); Serial.print(calData.magMin[2]);
      Serial.print(" maxZ: "); Serial.print(calData.magMax[2]); Serial.println();
    }
  }
  
  if (Serial.available()) {
    if (Serial.read() == 's') {                  // save the data
      calData.magValid = true;
      calLibWrite(0, &calData);
      Serial.print("Mag cal data saved for device "); Serial.println(imu->IMUName());
    }
  }
}