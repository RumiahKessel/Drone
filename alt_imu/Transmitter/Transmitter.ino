// Include RadioHead Amplitude Shift Keying Library
#include <RH_ASK.h>
// Include dependant SPI Library 
#include <SPI.h> 
 
// Create Amplitude Shift Keying Object
//receiver: 4 transmitter: 5
RH_ASK rf_driver(2000, 5, 4, 0); // ESP8266 or ESP32: do not use pin 11 or 2
 
void setup() {   
  Serial.begin(9600);
  // Initialize ASK Object
  if (!rf_driver.init()) {
    Serial.println("Failed to initialize driver");
  }
}
 
void loop()
{
    float x = 1;
    float y = 2;
    float alt = 100;
    float rot = 5;
    Serial.println("Send message");
    float controlData[4] = {x, y, alt, rot};

    // Convert floats to bytes
    uint8_t byteData[sizeof(float) * 4];
    memcpy(byteData, controlData, sizeof(byteData));

    rf_driver.send(byteData, sizeof(byteData));
    if (rf_driver.waitPacketSent())
    {
        Serial.println("Message sent successfully");
    }
    else
    {
        Serial.println("Error sending message");
    }
    delay(1000);
}

