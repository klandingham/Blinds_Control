/**
 * Tests output to serial monitor.
 *
 * NOTE: Using platformio integration in Eclipse, AFAIK THERE IS NO BUTTON TO OPEN THE SERIAL MONITOR.
 *       To work around this, right-click on any folder in the Project Explorer and select
 *       "Show in Local Terminal". At the prompt, enter "pio device monitor". You can also list
 *       devices with: "pio device list".
 *
 */
#include "CodeSelect.h"
#ifdef SERIAL_TEST
#include <Arduino.h>

const uint8_t LED_PIN = LED_BUILTIN;
const uint16_t DELAY = 1000;
void setup() {
    // serial monitor @ 9600 baud
    Serial.begin(9600);
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
  Serial.println("ON");
  digitalWrite(LED_PIN, HIGH);
  delay(DELAY);
  Serial.println("OFF");
  digitalWrite(LED_PIN, LOW);
  delay(DELAY);
} // loop()

#endif
