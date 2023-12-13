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
// #include "LPS.h"

RTIMU *imu;                                           // the IMU object
RTPressure *pressure;                                 // the pressure object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object
// LPS barometer;

//  DISPLAY_INTERVAL sets the rate at which results are displayed

#define DISPLAY_INTERVAL  1                         // interval between pose displays

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port

#define  SERIAL_PORT_SPEED  115200

#define VIRTUAL_TEMP 305.0f //in [K]

unsigned long lastDisplay;
unsigned long lastRate;
int sampleCount;


#define NUM_PRESSURE_READINGS 20

#define MBAR_ATT_SETTING 1013.25

float initial_pressure;
float initial_temp;

float report_pressure[NUM_PRESSURE_READINGS];
int pressure_index;
float sum_pressure;
float current_pressure;
float current_meters;
float base_alt;
float filteredAltitude;



void setup()
{
  int errcode;

  Serial.begin(SERIAL_PORT_SPEED);
  Wire.begin();
  imu = RTIMU::createIMU(&settings);                        // create the imu object
  pressure = RTPressure::createPressure(&settings); 

  Serial.print("ArduinoIMU starting using device "); Serial.println(imu->IMUName());
  if ((errcode = imu->IMUInit()) < 0) {
      Serial.print("Failed to init IMU: "); Serial.println(errcode);
  }

  if ((errcode = pressure->pressureInit()) < 0) {
      Serial.print("Failed to init pressure sensor: "); Serial.println(errcode);
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

  calibrate();
}

void calibrate() {

  pressure_index = 0;
  sum_pressure = 0;
  float latestPressure;
  float latestTemperature;
  int b = 0;
  int f = 0;


  while (f < 3) { //to flush out bad sensor readings
    if (pressure->pressureRead(latestPressure, latestTemperature)) {
      f += 1;
    }
  }

  while (b < NUM_PRESSURE_READINGS) {
    if (pressure->pressureRead(latestPressure, latestTemperature)) {
      report_pressure[b] = latestPressure;
      sum_pressure += report_pressure[b];
      b += 1;
      // Serial.print("index"); Serial.println(b);
      // Serial.print("pressure"); Serial.println(latestPressure);
      
    }
  }

  // base_alt = HypsometricEquation((sum_pressure/NUM_PRESSURE_READINGS) /1000);
  base_alt = pressureToAltitudeMeters(sum_pressure / NUM_PRESSURE_READINGS);
  Serial.print("base_alt "); Serial.println(base_alt);

  filteredAltitude = 0;

}

float pressureToAltitudeMeters(float pressure_mbar) {
  return (1 - pow(pressure_mbar / MBAR_ATT_SETTING, 0.190263)) * 44330.8;
}

/**
 * Use the Hypsometric equation to compute a change in altitude [m] from
 * a change in pressure [Pa].
 * 
 * @param currentPres  Current pressure reading [Pa]
 */
float HypsometricEquation(float currentPres) {
    float R = 287.0f;  // Gas const. [J / kg.K]
    float g = 9.81f;  // Grav. accel. [m/s/s]
    // Serial.println(sum_pressure/NUM_PRESSURE_READINGS/1000);
    return ((R * VIRTUAL_TEMP) / g) * log(sum_pressure/NUM_PRESSURE_READINGS/1000 / currentPres);  // [m]
}

void loop()
{  
  unsigned long now = millis();
  unsigned long delta;
  float latestPressure, latestTemperature;
  int loopCount = 1;


//   while (imu->IMURead()) {                                // get the latest data if ready yet
//       // this flushes remaining data in case we are falling behind
//       if (++loopCount >= 10)
//           continue;
//       fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
//       sampleCount++;
// //        if ((delta = now - lastRate) >= 1000) {
// ////            Serial.print("Sample rate: "); Serial.print(sampleCount);
// //            if (imu->IMUGyroBiasValid())
// //                Serial.println(", gyro bias valid");
// //            else
// //                Serial.println(", calculating gyro bias");
// //        
// //            sampleCount = 0;
// //            lastRate = now;
// //        }

//       if ((now - lastDisplay) >= DISPLAY_INTERVAL) {
//           lastDisplay = now;
// //          RTMath::display("Gyro:", (RTVector3&)imu->getGyro());                // gyro data
// //          RTMath::display("Accel:", (RTVector3&)imu->getAccel());              // accel data
// //          RTMath::display("Mag:", (RTVector3&)imu->getCompass());              // compass data
//           RTMath::displayRollPitchYaw("Pose:", (RTVector3&)fusion.getFusionPose()); // fused output
//           Serial.println();


//       }


//     }
    int b = 0;
    float alpha = 0.1;
    while (b < 5) {
      if (pressure->pressureRead(latestPressure, latestTemperature)) {
        sum_pressure -= report_pressure[pressure_index];
        Serial.print(" pressure read"); Serial.println(latestPressure);
        report_pressure[pressure_index] = latestPressure;
        sum_pressure += report_pressure[pressure_index];

        pressure_index = ((int) pressure_index + 1) % (NUM_PRESSURE_READINGS);
        b += 1;
      }
    }
    current_pressure = sum_pressure / NUM_PRESSURE_READINGS;
    Serial.print(" current_pressure"); Serial.println(current_pressure);
    current_meters = pressureToAltitudeMeters(current_pressure)-base_alt;
    // current_meters = HypsometricEquation(current_pressure / 1000);
    // filteredAltitude = alpha * current_meters + (1 - alpha) * filteredAltitude; //filtering
    // Serial.print("base alt"); Serial.println(base_alt);
    // Serial.print("current_meters "); Serial.println(filteredAltitude);
    // delay(200);
    // Serial.print("current pressure"); Serial.println(latestPressure);
    // Serial.print("current_temp "); Serial.println(latestTemperature);

    // current_meters = sum_pressure / NUM_PRESSURE_READINGS - base_alt;
    Serial.print("current_meters "); Serial.println(current_meters);

  
  }



