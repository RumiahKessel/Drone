#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>
#include <LPS.h>

#include <math.h>

#define LSM303D_SDA_PIN 21
#define LSM303D_SCL_PIN 22

LSM303 compass;
L3G gyro;
LPS barometer;

char report[80]; //how many do we want to keep track of
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
}

void calibrate() {
  //would need to use something to calibrate..
}

void loop() {
  // put your main code here, to run repeatedly:
  compass.read();
  gyro.read();

  float pressure = barometer.readPressureInchesHg();
  float altitude = barometer.pressureToAltitudeFeet(pressure);
  float temperature = barometer.readTemperatureF();
  Serial.println();
  Serial.println("Accelerometer: ");
  Serial.print(compass.a.x);
  Serial.print(", ");
  Serial.print(compass.a.y);
  Serial.print(", ");
  Serial.print(compass.a.z);
  Serial.println();
  Serial.println("Magnetometer: ");
  Serial.print(compass.m.x);
  Serial.print(", ");
  Serial.print(compass.m.y);
  Serial.println();
  Serial.println("Gyroscope: ");
  Serial.print(gyro.g.x);
  Serial.print(", ");
  Serial.print(gyro.g.y);
  Serial.print(", ");
  Serial.print(gyro.g.z);
  Serial.println();
  Serial.println("Barometer: ");
  Serial.print(pressure);
  Serial.print(", ");
  Serial.print(altitude);
  Serial.print(", ");
  Serial.print(temperature);
  delay(1000);
}
