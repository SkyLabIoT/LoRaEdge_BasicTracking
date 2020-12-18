/*
   For updating the LR1110 standard firmware to modem firmware
   Requires UPDATELRCHIP.py to run in a python enviroment on a host computer to receive the firmware package
*/

#include <skylabLR.h>

void setup() {
  /*Set serial monitor*/
  Serial.begin(9600);
  /*Set pinmodes*/
  pinMode (NSS, OUTPUT);
  pinMode (LEDR, OUTPUT);
  pinMode (LEDG, OUTPUT);
  pinMode (LEDB, OUTPUT);
  pinMode (busyPin, INPUT);
  pinMode (RESET, OUTPUT);
  digitalWrite (LEDR, 1);
  digitalWrite (LEDG, 1);
  digitalWrite (LEDB, 1);
  /*Start SPI*/
  SPI.begin();
  /*Init the LR1110 transceiver*/
  LR1110_init1();
  delay (1000);
  /*Update modem*/
  lr1110_update_trx_to_modem ();
}
void loop() {
  digitalWrite (LEDR, 1); 
  digitalWrite (LEDG, 0); //keep green led ON when done
  digitalWrite (LEDB, 1); 
}

void LR1110_init1() {
  digitalWrite(RESET, HIGH);
  delay(3000);
}

void lr1110_update_trx_to_modem()
{
  delay(1000);
  lr1110_bootloader_version_t version;
  lr1110_modem_version_t      modem;
  lr1110_modem_event_t        lr1110_modem_event;
  uint32_t combination[65];

  /*Put LR1110 in bootloader mode*/
  pinMode (busyPin, OUTPUT);
  digitalWrite(busyPin, 0);
  digitalWrite(RESET, 0);
  delay(1);
  digitalWrite(RESET, 1);
  delay(250);
  pinMode (busyPin, INPUT);
  delay(1000);
  
  digitalWrite (NSS, 1);
  while ( digitalRead (busyPin) == 1) {}
  lr1110_bootloader_get_version( &version );

  /*Erase Flash*/
  lr1110_bootloader_erase_flash();
  lr1110_status_t status           = LR1110_STATUS_OK;
  uint32_t        remaining_length = 61320;
  uint32_t        local_offset     = 0;
  digitalWrite (LEDB, 0);
  while ( ( remaining_length != 0 ) && ( status == LR1110_STATUS_OK ) )
  {
    for (int j = 0; j < 64; j++) {

      int incomingByte0 = 0; // for incoming serial data
      int incomingByte1 = 0; // for incoming serial data
      int incomingByte2 = 0; // for incoming serial data
      int incomingByte3 = 0; // for incoming serial data
      while (Serial.available() == 0) {
        Serial.print(0xFF);
        delay(1);
      };
      if (Serial.available() > 0) {
        digitalWrite (LEDB, 1); //turn blue led off
        //read the incoming byte:
        incomingByte0 = Serial.read();
        Serial.print(0xFF);
        while (Serial.available() == 0);
        if (Serial.available() > 0) {
          //read the incoming byte:
          incomingByte1 = Serial.read();
          Serial.print(0xFF);
          while (Serial.available() == 0);
          if (Serial.available() > 0) {
            //read the incoming byte:
            incomingByte2 = Serial.read();
            Serial.print(0xFF);
            while (Serial.available() == 0);
            if (Serial.available() > 0) {
              //read the incoming byte:
              incomingByte3 = Serial.read();
            }
          }
        }
        combination[j] = (incomingByte0 << 24 | incomingByte1 << 16 | incomingByte2 << 8 | incomingByte3); //combine the 4 bytes to 1 32 bit int
      }
    }

    status = ( lr1110_status_t ) lr1110_bootloader_write_flash_encrypted(  local_offset, combination, min( remaining_length, 64 ) );  //write the chunk of 64 32bit int to the LR1110
    digitalWrite (LEDR, 0);
    delay (5);
    digitalWrite (LEDR, 1);  //flash red LED
    local_offset += 256;
    remaining_length = ( remaining_length < 64 ) ? 0 : ( remaining_length - 64 );
  }
  /*Restart the LR1110 to confirm firmware update*/
  digitalWrite (LEDR, 1);
  delay( 100 );
  digitalWrite (LEDR, 0);
  lr1110_hal_reset( );
  digitalWrite (LEDR, 1);
  delay( 1500 );
  digitalWrite (LEDR, 0);
  lr1110_modem_get_version( &modem );
  delay( 200 );
  digitalWrite (LEDR, 1);
  digitalWrite (LEDG, 0); //keep green led ON when done
}
