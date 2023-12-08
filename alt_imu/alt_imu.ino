#include "Wire.h"
#include <LSM303.h>
#include <L3G.h>
#include <LPS.h>
#include <esp_timer.h>
#include <math.h>
#include "ESP32TimerInterrupt.h"

#define LSM303D_SDA_PIN 21
#define LSM303D_SCL_PIN 22

#define NUM_REPORT 50 

LSM303 compass;
L3G gyro;
LPS barometer;

float base_alt;

char report[200];

float magnet_report[3][NUM_REPORT];

float magnet_sum[3];

float pressure_report[NUM_REPORT];

float sum_pressure;

float sum_temp;

float temp_report[NUM_REPORT];

int report_idx;

float avg_magnet[3];
float avg_pressure;
float avg_temp;

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

  // compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  // compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};

  calibrate();
  report_idx = 0;
}

void calibrate() {
  Serial.println("Calibrating...");
  delay(5000); 
  // for (int i = 0; i < NUM_REPORT; ++i) {
  //   // magnet_report[0][i] = 1;
  //   // magnet_report[1][i] = 1;
  //   // magnet_report[2][i] = 1;
  //   // magnet_sum[0] += magnet_report[0][i]; 
  //   // magnet_sum[1] += magnet_report[1][i];
  //   // magnet_sum[2] += magnet_report[2][i];
  // }
  int num_readings = 100;
  for (int i = 0; i < num_readings; ++i) {
    sum_pressure += barometer.readPressureMillibars();
    delay(10); 
  }
    
  float base_pressure = sum_pressure / num_readings;
  base_alt = barometer.pressureToAltitudeMeters(base_pressure);
  Serial.println("base alt: ");
  Serial.println(base_alt);
}

void loop() {
  // readInMagnetValues();
  // readInPressureTemp();
  Serial.println("magnetic loop");
  Serial.println("MAG");
  // Serial.println(avg_magnet[0]);
  // Serial.println(avg_magnet[1]);
  // Serial.println(avg_magnet[2]);

  compass.read();

  Serial.println(headingCalc(compass.m.y, compass.m.z));
  Serial.println(headingCalc(compass.m.z, compass.m.x));
  Serial.println(headingCalc(compass.m.x, compass.m.y));
    
  report_idx = (report_idx + 1) % NUM_REPORT;
  delay(100);
}

float sumArray(float arr[], int size) {
  float sum = 0.0;
  for (int i = 0; i < size; ++i) {
    sum += arr[i];
  }
  return sum;
}

void readInMagnetValues() {
  compass.read();

  magnet_sum[0] -= magnet_report[0][report_idx];
  magnet_sum[1] -= magnet_report[1][report_idx];
  magnet_sum[2] -= magnet_report[2][report_idx];

  magnet_report[0][report_idx] = headingCalc(compass.m.y, compass.m.z);
  magnet_report[1][report_idx] = headingCalc(compass.m.z, compass.m.x);
  magnet_report[2][report_idx] = headingCalc(compass.m.x, compass.m.y);

  magnet_sum[0] += magnet_report[0][report_idx];
  magnet_sum[1] += magnet_report[1][report_idx];
  magnet_sum[2] += magnet_report[2][report_idx];

  avg_magnet[0] = magnet_sum[0] / NUM_REPORT;
  avg_magnet[1] = magnet_sum[1] / NUM_REPORT;
  avg_magnet[2] = magnet_sum[2] / NUM_REPORT;
  
}

void readInPressureTemp() {
  float pressure = barometer.readPressureMillibars();
  float altitude = barometer.pressureToAltitudeMeters(pressure);
  altitude -= base_alt;
  float temperature = barometer.readTemperatureF();

  sum_pressure -= pressure_report[report_idx];
  pressure_report[report_idx] = altitude;
  sum_pressure += pressure_report[report_idx];

  avg_pressure = sum_pressure / NUM_REPORT;

  sum_temp -= temp_report[report_idx];
  temp_report[report_idx] = temperature;
  sum_temp += temp_report[report_idx];

  avg_temp = sum_temp / NUM_REPORT;
}


float headingCalc(float int1, float int2) {
  return atan2(int1, int2)*180 / M_PI;
}
