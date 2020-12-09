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
    Serial.print("JOIN ERROR: ");             //print the error when join was not succesfull
    Serial.println(modem_response_code, HEX);
  }
  Serial.println("JOIN OK");
}

uint8_t adr_custom_list[16] = { 0x05, 0x05, 0x05, 0x04, 0x04, 0x04, 0x03, 0x03, 0x03, 0x02, 0x02, 0x02, 0x01, 0x01, 0x00, 0x00};

void LR1110_rejoin() {
  lr1110_modem_response_code_t modem_response_code;
  Serial.println("SET REG MODE");
  modem_response_code = lr1110_modem_system_set_reg_mode(LR1110_MODEM_SYSTEM_REG_MODE_DCDC);  //set regulator mode to DCDC
  Serial.println("SET TXCO");
  modem_response_code = lr1110_modem_system_set_tcxo_mode(LR1110_MODEM_SYSTEM_TCXO_CTRL_1_8V, BOARD_TCXO_WAKEUP_TIME * 1000 / 30.52 );  //set the TCXO oscilator
  Serial.println("SET RF SWITCH");
  lr1110_modem_system_rf_switch_cfg_t rf_switch_cfg;
  rf_switch_cfg.enable  = LR1110_MODEM_SYSTEM_RFSW0_HIGH | LR1110_MODEM_SYSTEM_RFSW1_HIGH | LR1110_MODEM_SYSTEM_RFSW2_HIGH;
  rf_switch_cfg.standby = 0;
  /* LoRa SPDT */
  rf_switch_cfg.rx = LR1110_MODEM_SYSTEM_RFSW0_HIGH;
  rf_switch_cfg.tx = LR1110_MODEM_SYSTEM_RFSW0_HIGH | LR1110_MODEM_SYSTEM_RFSW1_HIGH;
  rf_switch_cfg.tx_hp = LR1110_MODEM_SYSTEM_RFSW1_HIGH;
  /* GNSS LNA ON */
  rf_switch_cfg.gnss =  LR1110_MODEM_SYSTEM_RFSW2_HIGH;
  rf_switch_cfg.wifi = 0;
  modem_response_code = lr1110_modem_system_set_dio_as_rf_switch(&rf_switch_cfg); //set the radio switch IO pins on the LR1110
  Serial.println("SET TX POWER OFFSET");
  modem_response_code = lr1110_modem_set_tx_power_offset (0); //set the tx power offset
  Serial.println("SET RF OUTPUT");
  modem_response_code = lr1110_modem_set_rf_output(LR1110_MODEM_RADIO_PA_SEL_LP_HP_LF); //set the rf output
  Serial.println("SET DIO RF SWITCH");
  modem_response_code = lr1110_modem_set_class(LR1110_LORAWAN_CLASS_A); //set the lorawann class to A (only receive donwlink after a recent uplink)
  Serial.println("SET CLASS");
  modem_response_code = lr1110_modem_set_region(LR1110_LORAWAN_REGION_EU868); //set region
  Serial.println("SET REGION");
  modem_response_code = lr1110_modem_activate_duty_cycle(LORAWAN_DUTYCYCLE_ON); //set lorawan dutycycle rules
  Serial.println("SET DUTYCYCLE");
  modem_response_code = lr1110_modem_set_adr_profile(LORAWAN_DEFAULT_DATARATE, adr_custom_list);  //set detarate
  Serial.println("SET ADR PROFILE");
  modem_response_code = lr1110_modem_set_dm_port(1);  //set the port for the dm filed message
  Serial.println("SET DM PORT");
  lr1110_modem_dm_info_fields_t dm_info_fields;
  //dm_info_fields.dm_info_field[0] = LR1110_MODEM_DM_INFO_TYPE_REGION;
  dm_info_fields.dm_info_field[1] = LR1110_MODEM_DM_INFO_TYPE_UPTIME ;
  // dm_info_fields.dm_info_field[2] = LR1110_MODEM_DM_INFO_TYPE_TEMPERATURE;
  dm_info_fields.dm_info_length   = 1;
  modem_response_code = lr1110_modem_set_dm_info_field(&dm_info_fields); //set dm field information
  Serial.println("SET DM INFO FIELD");
  modem_response_code = lr1110_modem_set_dm_info_interval(LR1110_MODEM_REPORTING_INTERVAL_IN_HOUR, 24);  //set the interval of the dm field message
  Serial.println("SET DM INFO INTERVAL");
  modem_response_code = lr1110_modem_set_connection_timeout(255, 2400); //set connection timeout for receiving downlinks
  Serial.println("SET CONN TIMEOUT");
  LR1110_join();
  delay(100);
}

void LR1110_init() {
  lr1110_modem_response_code_t modem_response_code;
  Serial.println("RESET");
  lr1110_modem_hal_reset(); //reset the modem
  lr1110_modem_reset();
  digitalWrite(RESET, LOW);
  delay(10);
  digitalWrite(RESET, HIGH);
  Serial.println("LR1110 INIT");
  /*lr1110_bootloader_version_t version;
    lr1110_bootloader_get_version( &version );
    Serial.println( "LR1110:");
    Serial.print( "hw: ");
    Serial.println(version.hw, HEX);
    Serial.print( "type: ");
    Serial.println(version.type, HEX);
    Serial.print( "fw: ");
    Serial.println(version.fw, HEX);
    while (version.type == 0xDF) {
    lr1110_hal_reset( );
    delay( 1500 );
    lr1110_bootloader_get_version( &version );
    Serial.println( "LR1110:");
    Serial.print( "hw: ");
    Serial.println(version.hw, HEX);
    Serial.print( "type: ");
    Serial.println(version.type, HEX);
    Serial.print( "fw: ");
    Serial.println(version.fw, HEX);
    }*/
  Serial.println("SET PINS");
  digitalWrite(RESET, HIGH);
  digitalWrite(LNA, LOW);
  while ( digitalRead (busyPin) == 0) {}  //wait for busy pin to be high
  Serial.println("SET REG MODE");
  modem_response_code = lr1110_modem_system_set_reg_mode(LR1110_MODEM_SYSTEM_REG_MODE_DCDC);  //set regulator mode to DCDC
  Serial.println("SET TXCO");
  modem_response_code = lr1110_modem_system_set_tcxo_mode(LR1110_MODEM_SYSTEM_TCXO_CTRL_1_8V, BOARD_TCXO_WAKEUP_TIME * 1000 / 30.52 );  //set the TCXO oscilator
  Serial.println("SET RF SWITCH");
  lr1110_modem_system_rf_switch_cfg_t rf_switch_cfg;
  rf_switch_cfg.enable  = LR1110_MODEM_SYSTEM_RFSW0_HIGH | LR1110_MODEM_SYSTEM_RFSW1_HIGH | LR1110_MODEM_SYSTEM_RFSW2_HIGH;
  rf_switch_cfg.standby = 0;
  /* LoRa SPDT */
  rf_switch_cfg.rx = LR1110_MODEM_SYSTEM_RFSW0_HIGH;
  rf_switch_cfg.tx = LR1110_MODEM_SYSTEM_RFSW0_HIGH | LR1110_MODEM_SYSTEM_RFSW1_HIGH;
  rf_switch_cfg.tx_hp = LR1110_MODEM_SYSTEM_RFSW1_HIGH;
  /* GNSS LNA ON */
  rf_switch_cfg.gnss =  LR1110_MODEM_SYSTEM_RFSW2_HIGH;
  //rf_switch_cfg.wifi = 0;
  modem_response_code = lr1110_modem_system_set_dio_as_rf_switch(&rf_switch_cfg); //set the radio switch IO pins on the LR1110
  Serial.println("SET TX POWER OFFSET");
  modem_response_code = lr1110_modem_set_tx_power_offset (0); //set the tx power offset
  Serial.println("SET RF OUTPUT");
  modem_response_code = lr1110_modem_set_rf_output(LR1110_MODEM_RADIO_PA_SEL_LP_HP_LF); //set the rf output
  Serial.println("SET DIO RF SWITCH");
  modem_response_code = lr1110_modem_set_class(LR1110_LORAWAN_CLASS_A); //set the lorawann class to A (only receive donwlink after a recent uplink)
  Serial.println("SET CLASS");
  modem_response_code = lr1110_modem_set_region(LR1110_LORAWAN_REGION_EU868); //set region
  Serial.println("SET REGION");
  modem_response_code = lr1110_modem_activate_duty_cycle(LORAWAN_DUTYCYCLE_ON); //set lorawan dutycycle rules
  Serial.println("SET DUTYCYCLE");
  modem_response_code = lr1110_modem_set_adr_profile(LORAWAN_DEFAULT_DATARATE, adr_custom_list);  //set detarate
  Serial.println("SET ADR PROFILE");
  modem_response_code = lr1110_modem_set_dm_port(1);  //set the port for the dm filed message
  Serial.println("SET DM PORT");
  lr1110_modem_dm_info_fields_t dm_info_fields;
  //dm_info_fields.dm_info_field[0] = LR1110_MODEM_DM_INFO_TYPE_REGION;
  dm_info_fields.dm_info_field[1] = LR1110_MODEM_DM_INFO_TYPE_UPTIME ;
  //dm_info_fields.dm_info_field[2] = LR1110_MODEM_DM_INFO_TYPE_TEMPERATURE;
  dm_info_fields.dm_info_length   = 1;
  modem_response_code = lr1110_modem_set_dm_info_field(&dm_info_fields); //set dm field information
  Serial.println("SET DM INFO FIELD");
  modem_response_code = lr1110_modem_set_dm_info_interval(LR1110_MODEM_REPORTING_INTERVAL_IN_HOUR, 24);  //set the interval of the dm field message
  Serial.println("SET DM INFO INTERVAL");
  modem_response_code = lr1110_modem_set_connection_timeout(255, 2400); //set connection timeout for receiving downlinks
  Serial.println("SET CONN TIMEOUT");
}
