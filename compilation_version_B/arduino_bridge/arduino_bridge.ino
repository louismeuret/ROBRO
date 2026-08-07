#include <SoftwareSerial.h>

// STM32 hardware USART2 TX on PB3 -> Arduino pin 2 (RX)
// Common ground between boards.
SoftwareSerial bridge(2, 3); // RX on pin 2, TX on pin 3 (unused)

void setup()
{
  Serial.begin(57600);    // USB serial to the PC
  bridge.begin(57600);    // STM32 USART2 serial
}

void loop()
{
  if (bridge.available())
  {
    Serial.write(bridge.read());
  }
}
