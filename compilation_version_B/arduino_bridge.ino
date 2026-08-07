#include <SoftwareSerial.h>

// STM32 bit-banged serial TX on PA7 -> Arduino pin 2 (RX)
// Common ground between boards.
SoftwareSerial bridge(2, 3); // RX on pin 2, TX on pin 3 (unused)

void setup()
{
  Serial.begin(57600);    // USB serial to the PC
  bridge.begin(57600);    // STM32 bit-banged serial
}

void loop()
{
  if (bridge.available())
  {
    Serial.write(bridge.read());
  }
}
