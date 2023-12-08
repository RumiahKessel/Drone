// Include RadioHead Amplitude Shift Keying Library
#include <RH_ASK.h>
// Include dependant SPI Library 
#include <SPI.h> 
 
// Create Amplitude Shift Keying Object
RH_ASK rf_driver(2000, 4, 5, 0); // ESP8266 or ESP32: do not use pin 11 or 2
 
void setup()
{
  // Setup Serial Monitor
  Serial.begin(9600);

  // Initialize ASK Object
  if (!rf_driver.init()) {
    Serial.println("Failed to initialize driver");
  }
}
 
//Max length can send is 60 bytes
void loop()
{
    // Set buffer to size of expected message
    uint8_t byteData[sizeof(float) * 4];
    uint8_t buflen = sizeof(byteData);
    // Check if received packet is correct size
    if (rf_driver.recv(byteData, &buflen))
    {
      float receivedFloats[4];
      memcpy(receivedFloats, byteData, sizeof(byteData));
      
      // Message received with valid checksum
      Serial.print("Received Floats: ");
      for (int i = 0; i < 4; i++) {
        Serial.print(receivedFloats[i], 2); // Print with 2 decimal places
        Serial.print(" ");
      }
      Serial.println();
      // Serial.print("Message Received: ");
      // rf_driver.printBuffer("Got:", buf, buflen);
      // Serial.println(rf_driver.speed());
      // Serial.println((char*)buf);         
    } 
}