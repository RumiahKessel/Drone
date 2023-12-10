/*
 ESP8266 Receiver Code For 315MHz And 433MHz
 Wiring       
 ESP        Receiver
 3.3V   =    VCC
 GND    =    GND
 D3     =    Data
*/

#include <RCSwitch.h>
RCSwitch mySwitch = RCSwitch();
void setup() {
  Serial.begin(9600);
  mySwitch.enableReceive(0);  // Connect To GPIO0 (D3) On The ESP
}

void loop() {
  int value = mySwitch.getReceivedValue();
  if (mySwitch.available()) {
      Serial.print(" The Device Code Is: ");
      Serial.println(mySwitch.getReceivedValue());
      };
    // output(mySwitch.getReceivedValue(), mySwitch.getReceivedBitlength(), mySwitch.getReceivedDelay(), mySwitch.getReceivedRawdata(),mySwitch.getReceivedProtocol());
    mySwitch.resetAvailable();
  }