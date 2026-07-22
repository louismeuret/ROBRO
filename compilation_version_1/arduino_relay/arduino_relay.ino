/*
 * Relay: STM32 encoder position  ->  PC (USB serial).
 *
 * The STM32 (encoder_read_arduino.c) sends one ASCII line per reading on its
 * PB3 (USART2_TX) at 115200 8N1:
 *      "<full_mdeg>,<single_mdeg>\r\n"     (millidegrees, integers)
 * This sketch just forwards those bytes to the PC over USB. Open the Arduino
 * Serial Monitor (or read_serial.py) at 115200 to see them.
 *
 * Wiring:  STM32 PB3  ->  board RX pin       STM32 GND  ->  board GND
 *
 * --------------------------------------------------------------------------
 * ESP32 / Arduino Mega / Leonardo / any board with a spare hardware UART:
 * uses Serial1 (a real UART) -> reliable at 115200. Use this by default.
 * --------------------------------------------------------------------------
 */
void setup() {
  Serial.begin(115200);    // USB to PC
  Serial1.begin(115200);   // from STM32 PB3
  // ESP32 note: pick the RX pin explicitly, e.g.
  //   Serial1.begin(115200, SERIAL_8N1, /*RX=*/16, /*TX=*/17);
  // and wire STM32 PB3 -> GPIO16.
}

void loop() {
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }
}

/*
 * --------------------------------------------------------------------------
 * Arduino UNO / Nano (only ONE hardware UART, shared with USB):
 * use SoftwareSerial. It is NOT reliable at 115200 on a 16 MHz AVR, so set
 * BOTH sides to 38400 -- change ARDUINO_BAUD to 38400 in
 * encoder_read_arduino.c and use the sketch below instead of the one above.
 * --------------------------------------------------------------------------
 *
 * #include <SoftwareSerial.h>
 * SoftwareSerial link(10, 11);     // RX = D10 (wire to STM32 PB3), TX = D11 (unused)
 * void setup() {
 *   Serial.begin(38400);           // USB to PC
 *   link.begin(38400);             // from STM32 PB3
 * }
 * void loop() {
 *   while (link.available()) Serial.write(link.read());
 * }
 */
