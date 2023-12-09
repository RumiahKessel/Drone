#include "Wire.h"
#include <LSM303.h>
#include <L3G.h>
#include <LPS.h>
// #include <esp_timer.h>
#include <math.h>

#define LSM303D_SDA_PIN 21
#define LSM303D_SCL_PIN 22

#define DELAY_PERIOD 20 //ms

uint32_t cur;
uint32_t prev;

float ax;
float ay;
float az;


LSM303 compass;
L3G gyro;
LPS barometer;


float base_alt;

char report[200];

void setup() {
  Serial.begin(115200);
  Wire.begin(LSM303D_SDA_PIN, LSM303D_SCL_PIN);
  if (!compass.init()) {
    Serial.println("failed to detect compass");
    while(1);
  }
  if (!gyro.init()) {
    Serial.println("failed to detect gyro");
    while(1);
  }
  if (!barometer.init()) {
    Serial.println("failed to detect barometer");
    while(1);
  }
  compass.enableDefault();
  gyro.enableDefault();
  barometer.enableDefault();
  // calibrate();

  cur = 0;
  prev = 0;
  ax = 0.0f;
  ay = 0.0f;
  az = 0.0f;
}

void calibrate() {
  const int numReadings = 100;
  float sumPressure = 0.0;
  // Serial.println("Calibrating...");

  // Serial.println("Calibrate Accelerometer");
  // float sumAccel[3] = 0;
  // for (int i = 0; i < numReadings; ++i) {
  //   compass.read();
  //   sumAccel[3] += compass.a.x;
  //   delay(10); 
  // }
  delay(5000); 
  for (int i = 0; i < numReadings; ++i) {
    sumPressure += barometer.readPressureMillibars();
    delay(10); 
  }
  float base_pressure = sumPressure / numReadings;
  base_alt = barometer.pressureToAltitudeMeters(base_pressure);
  Serial.println("base alt: ");
  Serial.println(base_alt);
}

void loop() {

  cur = millis();  // Use millis() to perform readings
  if (cur - prev >= DELAY_PERIOD) {
    compass.read();
    Serial.print("Start"); Serial.print(",");
    Serial.print(compass.a.x); Serial.print(",");
    Serial.print(compass.a.y); Serial.print(",");
    Serial.print(compass.a.z); Serial.print(",");

    Serial.print(compass.m.x); Serial.print(",");
    Serial.print(compass.m.y); Serial.print(",");
    Serial.print(compass.m.z); Serial.print(",");

    Serial.println("End");

    //update timer
    prev = cur;
    

  }
}