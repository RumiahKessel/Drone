/*

OUTPUT COMMA SEPARATED MAGNETOMETER READINGS

Designed to work with Adafruit's FXOS8700 + FXAS21002 9-DOF IMU
module. See link below:
    https://www.adafruit.com/product/3463

Code By: Michael Wrona
Created: 14 Jan. 2021

Arduino Uno Pin Layout
----------------------
    (Sensor) - (Arduino)
    VIN - +3V3
    GND - GND
    SCL - A5
    SDA - A4

*/



#include <Wire.h>
#include <LSM303.h>


#define LSM303D_SDA_PIN 21
#define LSM303D_SCL_PIN 22


// Create accelerometer/magnetometer object
LSM303 compass;


void setup() {
    Serial.begin(115200);  // Baud rate

    Wire.begin(LSM303D_SDA_PIN, LSM303D_SCL_PIN);
  if (!compass.init()) {
    Serial.println("failed to detect compass");
    while(1);
  }
  compass.enableDefault();
  delay(100);
}



void loop()
{
    compass.read();

    // Transmit over serial
    Serial.print(compass.a.x); Serial.print(",");
    Serial.print(compass.a.y); Serial.print(",");
    Serial.println(compass.a.z);

    delay(100);  // 10 Hz
}