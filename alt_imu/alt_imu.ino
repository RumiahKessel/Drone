#include "Wire.h"
#include <LSM303.h>
#include <L3G.h>
#include <LPS.h>
#include <esp_timer.h>
#include <math.h>

#define LSM303D_SDA_PIN 21
#define LSM303D_SCL_PIN 22

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
  calibrate();
}

void calibrate() {
  const int numReadings = 100;
  float sumPressure = 0.0;
  Serial.println("Calibrating...");
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
  compass.read();
  gyro.read();

  float pressure = barometer.readPressureMillibars();
  float altitude = barometer.pressureToAltitudeMeters(pressure);
  altitude -= base_alt;
  float temperature = barometer.readTemperatureF();
  
  Serial.println();

  snprintf(report, sizeof(report), "Accel: %6d %6d %6d \nMag: %6d %6d %6d \nGyro: %6d %6d %6d \nBaro: %2f %2f %2f",
    compass.a.x, compass.a.y, compass.a.z,
    compass.m.x, compass.m.y, compass.m.z,
    gyro.g.x, gyro.g.y, gyro.g.z,
    pressure, altitude, temperature);
  Serial.println(report);
  
  delay(1000);
}

