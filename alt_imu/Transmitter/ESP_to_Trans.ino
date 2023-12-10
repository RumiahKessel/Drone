#include <RMT.h>

// Set the RMT channel number
#define RMT_CHANNEL 0

// Define the IR signal in an array of durations in microseconds
const uint16_t irSignal[] = {8000, 4500, 500, 550, 500, 550, 500, 550, 500, 1600, 500, 1600, 500, 1600, 500, 1600, 500, 550, 500, 550, 500, 550, 500, 1600, 500, 1600, 500, 1600, 500, 1600, 500, 1600, 500, 1600, 500, 1600, 500, 1600, 500, 550, 500, 550, 500, 550, 500, 1600, 500, 1600, 500, 550, 500, 550, 500, 550, 500, 550};

void setup() {
  // Initialize the RMT peripheral
  RMT.begin();

  // Set the output pin
  pinMode(RMT.txChannelToOutput(RMT_CHANNEL), OUTPUT);

  // Set the carrier frequency (38 kHz)
  RMT.addCarrier(RMT_CHANNEL, 38000);
}

void loop() {
  // Send the IR signal
  RMT.write(RMT_CHANNEL, irSignal, sizeof(irSignal) / sizeof(irSignal[0]));

  // Wait before sending the next signal
  delay(500);
}