////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib-Arduino
//
//  Copyright (c) 2014-2015, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <Wire.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h" 
#include "CalLib.h"
#include <EEPROM.h>
#include "RTPressure.h"

RTIMU *imu;                                           // the IMU object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object
RTPressure* barometer;

//  DISPLAY_INTERVAL sets the rate at which results are displayed

#define DISPLAY_INTERVAL  50                         // interval between pose displays

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port

#define  SERIAL_PORT_SPEED  115200

unsigned long lastDisplay;
unsigned long lastRate;
int sampleCount;


#define NUM_PRESSURE_READINGS 100

#define MBAR_ATT_SETTING 1013.25

float initial_pressure;
float initial_temp;

float report_pressure[NUM_PRESSURE_READINGS] = {0}; 
int pressure_index;
float sum_pressure;
float current_pressure;
float current_meters;
float base_alt;



void setup()
{
  int errcode;

  Serial.begin(SERIAL_PORT_SPEED);
  Wire.begin();
  imu = RTIMU::createIMU(&settings);                        // create the imu object

<<<<<<< HEAD
    // Slerp power controls the fusion and can be between 0 and 1
    // 0 means that only gyros are used, 1 means that only accels/compass are used
    // In-between gives the fusion mix.
    
    fusion.setSlerpPower(0.04);
    
    // use of sensors in the fusion algorithm can be controlled here
    // change any of these to false to disable that sensor
    
    fusion.setGyroEnable(true);
    fusion.setAccelEnable(true);
    fusion.setCompassEnable(true);
=======
  Serial.print("ArduinoIMU starting using device "); Serial.println(imu->IMUName());
  if ((errcode = imu->IMUInit()) < 0) {
      Serial.print("Failed to init IMU: "); Serial.println(errcode);
  }

  if (imu->getCalibrationValid())
      Serial.println("Using compass calibration");
  else
      Serial.println("No valid compass calibration data");

  lastDisplay = lastRate = millis();
  sampleCount = 0;

  // Slerp power controls the fusion and can be between 0 and 1
  // 0 means that only gyros are used, 1 means that only accels/compass are used
  // In-between gives the fusion mix.
  
  fusion.setSlerpPower(0.02);
  
  // use of sensors in the fusion algorithm can be controlled here
  // change any of these to false to disable that sensor
  
  fusion.setGyroEnable(true);
  fusion.setAccelEnable(true);
  fusion.setCompassEnable(true);

  //Pressure initialize
  barometer = RTPressure::createPressure(&settings);
  if ((errcode = barometer->pressureInit()) < 0) {
      Serial.print("Failed to init pressure sensor: "); Serial.println(errcode);
  }

  // calibrate();
>>>>>>> 5d4f5f1ad6b7a4076b40dd7214de7d3f14ef81e8
}

void calibrate() {

  pressure_index = 0;
  sum_pressure = 0;
  float latestPressure;
  float latestTemperature;
  int b = 0;
  while (barometer->pressureRead(latestPressure, latestTemperature)) {
    report_pressure[b] = latestPressure;
    Serial.print("latestPressure " ); Serial.println(latestPressure);
    sum_pressure += latestPressure;
    if (b >= NUM_PRESSURE_READINGS) {
      break;
    }
    b = (b + 1) % NUM_PRESSURE_READINGS;
  }
  

  base_alt = pressure_to_meters(sum_pressure / NUM_PRESSURE_READINGS);
  Serial.print("base_alt "); Serial.println(base_alt);


}

float pressure_to_meters(float pressure_mbar) {
  return (1 - pow(pressure_mbar / MBAR_ATT_SETTING, 0.190263)) * 44330.8;
}

// float low_pass_filter(float a, float b, float lpf) {
//   //first-order low-pass filter
//   return lpf * a + (1.0f - lpf) * b;
// }

void loop()
{  
    unsigned long now = millis();
    unsigned long delta;
    float latestPressure, latestTemperature;
    int loopCount = 1;
  
    while (imu->IMURead()) {                                // get the latest data if ready yet
        // this flushes remaining data in case we are falling behind
        if (++loopCount >= 10)
            continue;
        fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
        sampleCount++;
//        if ((delta = now - lastRate) >= 1000) {
////            Serial.print("Sample rate: "); Serial.print(sampleCount);
//            if (imu->IMUGyroBiasValid())
//                Serial.println(", gyro bias valid");
//            else
//                Serial.println(", calculating gyro bias");
//        
//            sampleCount = 0;
//            lastRate = now;
//        }

        

        if ((now - lastDisplay) >= DISPLAY_INTERVAL) {
            lastDisplay = now;
//          RTMath::display("Gyro:", (RTVector3&)imu->getGyro());                // gyro data
//          RTMath::display("Accel:", (RTVector3&)imu->getAccel());              // accel data
//          RTMath::display("Mag:", (RTVector3&)imu->getCompass());              // compass data
            RTMath::displayRollPitchYaw("Pose:", (RTVector3&)fusion.getFusionPose()); // fused output
           Serial.println();

          if (barometer->pressureRead(latestPressure, latestTemperature)) {
            Serial.println(latestPressure);
            // sum_pressure -= report_pressure[pressure_index];
            // report_pressure[pressure_index] = latestPressure;
            // sum_pressure += report_pressure[pressure_index];

            // pressure_index = ((int) pressure_index + 1) % (NUM_PRESSURE_READINGS);

            // current_pressure = sum_pressure / NUM_PRESSURE_READINGS;
            // current_meters = pressure_to_meters(current_pressure)-base_alt;
          }
          // Serial.print("Barometer "); Serial.println(current_meters);
        }


    }


}
