#include <SPI.h>
#include <stdlib.h>

#include "lr1110_modem_hal.h"
#include "lr1110_modem_system.h"
#include "lr1110_modem_lorawan.h"
#include "skylabLR.h"

uint8_t read_battery() {
  analogReadResolution(8);
  uint8_t batt = analogRead(batPin);  //read the analog battery value
  float voltage = (float)((3.3 / 255) * ((4.7 + 10) / 10) * batt);
  Serial.print("Battery voltage: ");
  Serial.println(voltage);
  return batt;
}

void LR1110_join() {
  lr1110_modem_response_code_t modem_response_code;
  modem_response_code = lr1110_modem_join();  //reqeust to join network
  while (modem_response_code != 0) {          //reqeust to join network again until a OK response code
    delay(10);
    modem_response_code = lr1110_modem_join();
    Serial.print("JOIN COMMAND ERROR: ");             //print the error when join was not succesfull
    Serial.println(modem_response_code, HEX);
  }
  Serial.println("JOIN COMMAND OK");
}
