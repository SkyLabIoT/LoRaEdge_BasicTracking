/*
   LoRa mode A device.
   Support for configuration via downlink .
   Interval with WiFi scan, sensor values and GNSS scan. Send in 2 payloads.
   Interval for control message.
   Information on serial monitor.
   Radio beacon not yet implemented.
*/

#include <skylabLR.h>
#include <ArduinoLowPower.h>
#include <RTCZero.h>

/* Set adresses. dev-eui is ignored by default and set by de factory default key, displayed in serial monitor. Pin not used by default*/
uint8_t join_eui[8]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t dev_eui[8]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t app_key[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint32_t pin;
uint16_t intervalTime = 1; // +/- minutes
uint16_t controlTime = 1415; // +/- minutes, control message in a bit less than every 24h

bool ledActivation = 1;
bool beaconActivation = 0;
uint16_t beaconTime = 10;

/*interrupt variables*/
uint32_t sendFreq = (intervalTime);
uint32_t controlFreq = (controlTime);
uint32_t beaconFreq = (beaconTime);
uint32_t counter = 0;
uint32_t counter2 = 0;
uint32_t counter3 = 0;

#define WIFI_MAX_BASIC_RESULTS_PER_SCAN 3 //Max adresses to scan for when wifi sniffing

#define WIFI_PAYLOAD_FORMAT 2 //define payload types for send function
#define GNSS_PAYLOAD_FORMAT 3

static lr1110_modem_uplink_type_t is_tx_confirmed = (lr1110_modem_uplink_type_t) LORAWAN_CONFIRMED_MSG_ON;
static uint8_t tx_frame_buffer[LORAWAN_APP_DATA_MAX_SIZE];

wifi_t wifi;
gnss_t gnss;
lr1110_modem_event_fields_t  event_fields;
lr1110_modem_event_t* lr1110_modem_event;
RTCZero rtc;

void setup() {
  /*Set serial monitor*/
  delay(30000); //start after 30 sec for allowing code to upload before disabling USB.
  Serial.begin(9600);
  Serial.println();
  Serial.println("BEGIN");
  /*Set pinmodes*/
  pinMode (NSS, OUTPUT);
  pinMode (snifLED, OUTPUT);
  pinMode (snifLED1, OUTPUT);
  pinMode (snifLED2, OUTPUT);
  pinMode (busyPin, INPUT);
  pinMode (RESET, OUTPUT);
  pinMode (LNA, OUTPUT);
  pinMode (eventPin, INPUT);
  pinMode (batPin, INPUT);
  digitalWrite(snifLED, HIGH);
  digitalWrite(snifLED1, HIGH);
  digitalWrite(snifLED2, HIGH);
  /*Start SPI*/
  SPI.begin();
  /*Init the LR1110 transceiver*/
  LR1110_init();
  Serial.println("LR KEYS");
  delay(100);
  Serial.print("Dev eui: ");
  lr1110_modem_response_code_t modem_response_code;
  modem_response_code = lr1110_modem_get_dev_eui(dev_eui);    //request the dev eui from the LR1110 and print in on the serial monitor
  //lr1110_modem_set_dev_eui(dev_eui);
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print(dev_eui[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  Serial.print("Join eui: ");
  modem_response_code = lr1110_modem_set_join_eui(join_eui);  //set the join eui and print in on the serial monitor
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print(join_eui[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  Serial.print("App key: ");
  modem_response_code = lr1110_modem_set_app_key(app_key);  //set the app key and print in on the serial monitor
  for (uint8_t i = 0; i < 16; i++) {
    Serial.print(app_key[i], HEX);
    Serial.print(" ");
  }
  lr1110_modem_get_pin(&pin); //request the PIN from the LR1110 and print it on the serial monitor
  Serial.println();
  Serial.print("PIN: ");
  Serial.print(pin, HEX);
  Serial.println();
  Serial.println("LR MODEM SET");
  delay(10);
  lr1110_modem_version_t modem;
  lr1110_modem_get_version( &modem ); //request the firmware information and print it on the serial monitor
  Serial.print( "LORAWAN     : " );
  Serial.println(modem.lorawan );
  Serial.print( "FIRMWARE    : " );
  Serial.println( modem.firmware );
  Serial.print( "BOOTLOADER  : ");
  Serial.println(modem.bootloader );
  delay(10);
  LR1110_join();
  LR1110_event_flush();                       //flush all modem events
  /*Set interrupt settings*/
  LowPower.attachInterruptWakeup(eventPin, LR1110_event_flush, RISING);
  rtc.begin();
  rtc.attachInterrupt(oneMinute);
  rtc.setAlarmSeconds(5);
  rtc.enableAlarm(rtc.MATCH_SS);
}
void loop() {//puts uC in sleepmode when nothing to do, remove these 2 commands if serial debugging is needed
  USBDevice.detach();
  LowPower.deepSleep();
}
void control_function() { //function for sending the control message
  lr1110_modem_status_t modemStat;
  lr1110_modem_get_status(&modemStat);
  Serial.print("Modem status: ");
  Serial.println(modemStat);
  if (modemStat != 0x08)
  {
    lr1110_modem_response_code_t modem_response_code;
    modem_response_code = lr1110_modem_leave_network();
    Serial.println("Modem ERROR, leaving network and try to rejoin");
    Serial.print("Leaving network response: ");
    Serial.println(modem_response_code);
    LR1110_rejoin();
    counter2 = (controlFreq); //retry next minute
  }
  else {
    static uint8_t tx_frame_buffer_control[LORAWAN_APP_DATA_MAX_SIZE];
    uint8_t batteryValue = read_battery();
    tx_frame_buffer_control[0] = batteryValue;
    lr1110_modem_request_tx(10, is_tx_confirmed, tx_frame_buffer_control, 1);
    Serial.println("Control message send");
    counter2 = 1;
  }
}
void main_function() {  //function to send the data
  uint8_t sendResp;
  if (ledActivation == 1) {
    digitalWrite(snifLED, LOW);
  } //turn led on when led activation is set to 1
  Serial.println();
  Serial.println("...SEND INTERRUPT...");
  lr1110_modem_status_t modemStat;
  lr1110_modem_get_status(&modemStat);
  Serial.print("Modem status: ");
  Serial.println(modemStat);
  if (modemStat != 0x08)
  {
    lr1110_modem_response_code_t modem_response_code;
    modem_response_code = lr1110_modem_leave_network();
    Serial.println("Modem ERROR, leaving network and try to rejoin");
    Serial.print("Leaving network response: ");
    Serial.println(modem_response_code);
    LR1110_rejoin();
    counter = (sendFreq); //retry next minute
  }
  else {
    if (ledActivation == 1) {
      digitalWrite(snifLED, LOW);
    } //turn led on when led activation is set to 1
    LR1110_WiFi_scan();   //scan for WiFi adresses
    digitalWrite(snifLED, HIGH); //turn the led off
    if (ledActivation == 1) {
      digitalWrite(snifLED2, LOW);
    } //turn led on when led activation is set to 1
    sendResp = send_payload(WIFI_PAYLOAD_FORMAT);  //send the WiFi payload to the lora network
    digitalWrite(snifLED2, HIGH); //turn the led off
    if (ledActivation == 1) {
      digitalWrite(snifLED1, LOW);
    } //turn led on when led activation is set to 1
    //LR1110_event_flush(); //flush the event register and check for downlink
    digitalWrite(LNA, HIGH);  //Turn on the Low Noise Amp for the GNSS antenna
    LR1110_GNSS_scan();     //scan for GNSS data
    digitalWrite(LNA, LOW);   //Turn off the Low Noise Amp for the GNSS antenna
    digitalWrite(snifLED1, HIGH); //turn the led off
    if (ledActivation == 1) {
      digitalWrite(snifLED2, LOW);
    } //turn led on when led activation is set to 1
    sendResp = send_payload(GNSS_PAYLOAD_FORMAT);  //send the GNSS payload to the lora network
    //LR1110_event_flush(); //flush the event register and check for downlink
    digitalWrite(snifLED2, HIGH); //turn the led off
    counter = 1;  //reset the counter
  }
}

void oneMinute () {
  if (counter >= sendFreq) {
    main_function(); //execute the main function
  }
  else {
    counter++;  //count up the counter
  }

  if (counter2 >= controlFreq) {
    control_function();
  }
  else {
    counter2++; //count up the counter
  }
}
uint8_t send_payload(uint8_t payloadKind) {
  lr1110_modem_response_code_t modem_response_code;
  if (payloadKind == WIFI_PAYLOAD_FORMAT) {
    if (wifi.results.nbrResults == 3) {
      tx_frame_buffer[0] = (wifi.results.results[0].rssi);
      tx_frame_buffer[1] = (wifi.results.results[0].mac_address[0]);
      tx_frame_buffer[2] = (wifi.results.results[0].mac_address[1]);
      tx_frame_buffer[3] = (wifi.results.results[0].mac_address[2]);
      tx_frame_buffer[4] = (wifi.results.results[0].mac_address[3]);
      tx_frame_buffer[5] = (wifi.results.results[0].mac_address[4]);
      tx_frame_buffer[6] = (wifi.results.results[0].mac_address[5]);
      tx_frame_buffer[7] = (wifi.results.results[1].rssi);
      tx_frame_buffer[8] = (wifi.results.results[1].mac_address[0]);
      tx_frame_buffer[9] = (wifi.results.results[1].mac_address[1]);
      tx_frame_buffer[10] = (wifi.results.results[1].mac_address[2]);
      tx_frame_buffer[11] = (wifi.results.results[1].mac_address[3]);
      tx_frame_buffer[12] = (wifi.results.results[1].mac_address[4]);
      tx_frame_buffer[13] = (wifi.results.results[1].mac_address[5]);
      tx_frame_buffer[14] = (wifi.results.results[2].rssi);
      tx_frame_buffer[15] = (wifi.results.results[2].mac_address[0]);
      tx_frame_buffer[16] = (wifi.results.results[2].mac_address[1]);
      tx_frame_buffer[17] = (wifi.results.results[2].mac_address[2]);
      tx_frame_buffer[18] = (wifi.results.results[2].mac_address[3]);
      tx_frame_buffer[19] = (wifi.results.results[2].mac_address[4]);
      tx_frame_buffer[20] = (wifi.results.results[2].mac_address[5]);
    }
    else if (wifi.results.nbrResults == 2) {
      tx_frame_buffer[0] = (wifi.results.results[0].rssi);
      tx_frame_buffer[1] = (wifi.results.results[0].mac_address[0]);
      tx_frame_buffer[2] = (wifi.results.results[0].mac_address[1]);
      tx_frame_buffer[3] = (wifi.results.results[0].mac_address[2]);
      tx_frame_buffer[4] = (wifi.results.results[0].mac_address[3]);
      tx_frame_buffer[5] = (wifi.results.results[0].mac_address[4]);
      tx_frame_buffer[6] = (wifi.results.results[0].mac_address[5]);
      tx_frame_buffer[7] = (wifi.results.results[1].rssi);
      tx_frame_buffer[8] = (wifi.results.results[1].mac_address[0]);
      tx_frame_buffer[9] = (wifi.results.results[1].mac_address[1]);
      tx_frame_buffer[10] = (wifi.results.results[1].mac_address[2]);
      tx_frame_buffer[11] = (wifi.results.results[1].mac_address[3]);
      tx_frame_buffer[12] = (wifi.results.results[1].mac_address[4]);
      tx_frame_buffer[13] = (wifi.results.results[1].mac_address[5]);
      for (int k = 14; k < 21; k++) {
        tx_frame_buffer[k] = 0;
      }
    }
    else if (wifi.results.nbrResults == 1) {
      tx_frame_buffer[0] = (wifi.results.results[0].rssi);
      tx_frame_buffer[1] = (wifi.results.results[0].mac_address[0]);
      tx_frame_buffer[2] = (wifi.results.results[0].mac_address[1]);
      tx_frame_buffer[3] = (wifi.results.results[0].mac_address[2]);
      tx_frame_buffer[4] = (wifi.results.results[0].mac_address[3]);
      tx_frame_buffer[5] = (wifi.results.results[0].mac_address[4]);
      tx_frame_buffer[6] = (wifi.results.results[0].mac_address[5]);
      for (int k = 6; k < 21; k++) {
        tx_frame_buffer[k] = 0;
      }
    }
    else if (wifi.results.nbrResults == 0) {
      for (int k = 0; k < 21; k++) {
        tx_frame_buffer[k] = 0;
      }
    }
    uint8_t batteryValue = read_battery();
    tx_frame_buffer[21] = (batteryValue);
    modem_response_code = lr1110_modem_request_tx(2, is_tx_confirmed, tx_frame_buffer, 22); //send over port 2, ack defined above, send the buffer, 22 bytes
    if (modem_response_code != 0) {
      Serial.print("SEND ERROR: ");     //error check
      Serial.println(modem_response_code, HEX);
      return 2;   //return 2 for SEND ERROR
    }
    else {
      Serial.println("SEND OK");
      return 1;   //return 1 for OK
    }

  }
  if (payloadKind == GNSS_PAYLOAD_FORMAT) {
    uint8_t GnssPaySize = 0;
    if (gnss.capture_result.result_size < 100) {
      GnssPaySize = gnss.capture_result.result_size;
    }
    else {
      GnssPaySize = 100;
    }
    for ( uint8_t i = 0; i < GnssPaySize; i++ ) {
      tx_frame_buffer[i] = gnss.capture_result.result_buffer[i];
    }
    modem_response_code = lr1110_modem_request_tx(3, is_tx_confirmed, tx_frame_buffer, GnssPaySize );//send over port 3, ack defined above, send the buffer, size of payload bytes
    if (modem_response_code != 0) {   //error check
      Serial.print("SEND ERROR: ");
      Serial.println(modem_response_code, HEX);
      return 2;     //return 2 for SEND ERROR
    }
    else {
      Serial.println("SEND OK");
      return 1;     //return 1 for OK
    }
  }
  else
  {
    return 3;       //return 3 for unexpected payload type
  }
}
void LR1110_event_flush() { //function flushes the open events in the LR1110 and reads the downlink if the event is of the donwlink type
  Serial.print("Modem event types: ");
  lr1110_modem_response_code_t modem_response_code;
  while ( digitalRead (eventPin) == 1 ) //only do something when eventpin is high and keep doing until pin goes low
  {
    modem_response_code = lr1110_modem_get_event( &event_fields );  //request event from register
    Serial.print(modem_response_code);
    Serial.print(" ");
    if (event_fields.event_type == LR1110_MODEM_LORAWAN_EVENT_DOWN_DATA) {  //if eventtype is down data do things with the received data
      Serial.println("Downlink event");
      int8_t  rssi  = ( ( int8_t ) event_fields.buffer[0] ) - 64;
      int8_t  snr   = ( event_fields.buffer[1] << 2 );
      uint8_t flags = event_fields.buffer[2];
      uint8_t port  = event_fields.buffer[3];
      uint8_t buffer_size = event_fields.buffer_len - 4;
      for ( uint8_t i = 0; i < buffer_size; i++ )
      {
        event_fields.buffer[i] = event_fields.buffer[i + 4];
      }
      Serial.print("RX data: ");
      for (int8_t i = 0; i < buffer_size; i++) {
        Serial.print (event_fields.buffer[i], HEX);
        Serial.print (" ");
      }
      Serial.println ();
      if (event_fields.buffer[0] == 0) {  //byte 0 contains ledACtivation, set ledActivation variable accordingly, or do noting when value is not 0 or 1
        ledActivation = 0;
      }
      if (event_fields.buffer[0] == 1) {
        ledActivation = 1;
      }
      else {}
      uint16_t intervalTimeRec = (event_fields.buffer[1] << 8 | event_fields.buffer[2]);  //read byte 1 and 2 for interval time
      if (intervalTimeRec > 0) {   //only configure new interval time when received time is higher than 0 minutes
        intervalTime = intervalTimeRec;
        sendFreq = (intervalTime);
      }
      if (event_fields.buffer[3] == 1) {  //byte 3 contains beaconActivation, turn beacon on when 1
        beaconActivation = 1;
      }
      uint16_t beaconTimeRec = (event_fields.buffer[4] << 8 | event_fields.buffer[5]);  //read byte 4 and 5 for beacon time
      if (beaconTimeRec > 0) {   //only configure new interval time when received time is higher than 0 minutes
        beaconTime = beaconTimeRec;
        beaconFreq = (beaconTime);
      }
      Serial.print("LED: ");
      Serial.println(ledActivation);
      Serial.print("Interval: ");
      Serial.println(sendFreq);
      Serial.print("Beacon: ");
      Serial.println(beaconActivation);
      Serial.print("Beacon time: ");
      Serial.println(beaconFreq);
    }
  }
  Serial.println();
  return;
}
void LR1110_WiFi_scan() {
  lr1110_modem_response_code_t modem_response_code;
  wifi.settings.enabled       = true;
  wifi.settings.channels      = 0x3FFF;  // by default enable all channels
  wifi.settings.types         = LR1110_MODEM_WIFI_TYPE_SCAN_B_G_N;
  wifi.settings.scan_mode     = LR1110_MODEM_WIFI_SCAN_MODE_BEACON_AND_PACKET;
  wifi.settings.nbr_retrials  = 5;
  wifi.settings.max_results   = WIFI_MAX_BASIC_RESULTS_PER_SCAN;
  wifi.settings.timeout       = 110;
  wifi.settings.result_format = LR1110_MODEM_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL;
  lr1110_modem_wifi_reset_cumulative_timing();    //set the WiFi settings
  lr1110_modem_wifi_cfg_hardware_debarker( true );  //set the cfg debarker for faster WiFi B scanning
  modem_response_code = lr1110_modem_wifi_passive_scan(
                          wifi.settings.types, wifi.settings.channels, wifi.settings.scan_mode,
                          wifi.settings.max_results, wifi.settings.nbr_retrials, wifi.settings.timeout,
                          true, wifi.settings.result_format );  //do a passive WiFi scan
  if (modem_response_code != 0) {
    Serial.print("WiFi scan ERROR: ");
    Serial.println(modem_response_code, HEX); //print error code when response is not expected
  }
  else {
    Serial.println("WiFi scan OK");
    Serial.println("Get WiFi event");
    while (digitalRead (busyPin) == 0) {}
    while (digitalRead (eventPin) == 0) {}    //wait for event and busy pin
    modem_response_code = lr1110_modem_get_event(&event_fields);
    if ( modem_response_code == LR1110_MODEM_RESPONSE_CODE_OK )
    {
      if (event_fields.event_type = LR1110_MODEM_LORAWAN_EVENT_WIFI_SCAN_DONE) {  //request the WiFi event data if wifi event occured, else print an not expected error
        Serial.println("WiFi event");
        wifi_scan_done( event_fields.buffer, event_fields.buffer_len );
      }
      else {
        Serial.println("Event type not expected");
      }
      lr1110_modem_wifi_cumulative_timings_t wifi_results_timings = { 0 };
      lr1110_modem_wifi_basic_mac_type_channel_result_t wifi_results_mac_addr[WIFI_MAX_BASIC_RESULTS_PER_SCAN] = { 0 }; //define buffer for wifi results
      uint8_t nb_result;                                                                                                //define variable for number of detected WiFi points
      lr1110_modem_wifi_read_cumulative_timing(&wifi_results_timings);  //set wifi cumulative timings
      lr1110_modem_wifi_read_basic_results( wifi.results.raw_buffer, wifi.results.raw_buffer_size, wifi_results_mac_addr, &nb_result ); //read the WiFi basic results to the buffer
      wifi_add_basic_mac_to_results( LR1110_MODEM_SYSTEM_REG_MODE_DCDC, &wifi.results, wifi_results_mac_addr, nb_result, wifi_results_timings );
      Serial.print ("Number of WiFi results: ");
      Serial.println (wifi.results.nbrResults);
      for (uint8_t i = 0; i < wifi.results.nbrResults; i++) {
        Serial.print ("WiFi point: ");
        Serial.print (i);
        Serial.print (" type: ");
        Serial.print(wifi.results.results[i].type);
        Serial.print (" data channel: ");
        Serial.print(wifi.results.results[i].channel);
        Serial.print (" rssi: ");
        Serial.print(wifi.results.results[i].rssi);
        Serial.print (" mac: ");
        Serial.print(wifi.results.results[i].mac_address[0], HEX);
        Serial.print(" ");
        Serial.print(wifi.results.results[i].mac_address[1], HEX);
        Serial.print(" ");
        Serial.print(wifi.results.results[i].mac_address[2], HEX);
        Serial.print(" ");
        Serial.print(wifi.results.results[i].mac_address[3], HEX);
        Serial.print(" ");
        Serial.print(wifi.results.results[i].mac_address[4], HEX);
        Serial.print(" ");
        Serial.println(wifi.results.results[i].mac_address[5], HEX);
      }
      Serial.println("WiFi scan end");
    }
    else
    {
      Serial.print("ERROR: ");
      Serial.println(modem_response_code, HEX);
    }
  }
}
void wifi_scan_done( uint8_t* buffer, uint16_t size ) { //function that puts wfi result in wifi buffer
  memcpy( wifi.results.raw_buffer, buffer, size );
  wifi.results.raw_buffer_size = size;
}
void wifi_add_basic_mac_to_results( lr1110_modem_system_reg_mode_t reg_mode, wifi_scan_all_result_t* results,
                                    lr1110_modem_wifi_basic_mac_type_channel_result_t* scan_result, uint8_t nbr_results,
                                    lr1110_modem_wifi_cumulative_timings_t timing ) {   //function that makes results readeble
  for ( uint8_t index = 0; index < nbr_results; index++ )
  {
    results->results[index].channel =
      lr1110_modem_extract_channel_from_info_byte( scan_result[index].channel_info_byte );
    results->results[index].type =
      lr1110_modem_extract_signal_type_from_data_rate_info( scan_result[index].data_rate_info_byte );
    memcpy( results->results[index].mac_address, scan_result[index].mac_address,
            LR1110_MODEM_WIFI_MAC_ADDRESS_LENGTH );
    results->results[index].rssi = scan_result[index].rssi;
  }
  results->timings = timing;
  results->nbrResults = nbr_results;
}
void LR1110_GNSS_scan() {
  lr1110_modem_response_code_t modem_response_code;
  uint8_t numberSat = 0;

  gnss.settings.enabled              = true;
  gnss.settings.constellation_to_use = LR1110_MODEM_GNSS_GPS_MASK | LR1110_MODEM_GNSS_BEIDOU_MASK;
  gnss.settings.scan_type            = AUTONOMOUS_MODE;
  gnss.settings.search_mode          = LR1110_MODEM_GNSS_OPTION_DEFAULT;
  gnss.settings.input_paramaters     = LR1110_MODEM_GNSS_BIT_CHANGE_MASK | LR1110_MODEM_GNSS_DOPPLER_MASK | LR1110_MODEM_GNSS_PSEUDO_RANGE_MASK;
  lr1110_modem_gnss_set_constellations_to_use(LR1110_MODEM_GNSS_GPS_MASK /*| LR1110_MODEM_GNSS_BEIDOU_MASK*/);    //set the GNSS settings
  gnss.settings.nb_sat = 6; //max satellites to return, when 0 all detected sattellites are returned
  modem_response_code = lr1110_modem_gnss_scan_autonomous_md(gnss.settings.input_paramaters, gnss.settings.nb_sat); //scan for GNSS data
  if (modem_response_code != 0) {
    Serial.print("GNSS scan ERROR: ");
    Serial.println(modem_response_code, HEX);               //print error if response code is not expected
  }
  else {
    Serial.println("GNSS scan OK");
    Serial.println("Get GNSS event");
    while (digitalRead (busyPin) == 0) {}
    while (digitalRead (eventPin) == 0) {}                  //wait for event and busy pins
    lr1110_modem_gnss_get_nb_detected_satellites(&numberSat); //request the number of detected satellites
    gnss.capture_result.nb_detected_satellites = numberSat;   //put number in buffer
    modem_response_code = lr1110_modem_get_event(&event_fields); //get the GNSS event
    lr1110_modem_gnss_get_detected_satellites(numberSat, gnss.capture_result.detected_satellites ); //get GNSS satellites information

    if ( modem_response_code == LR1110_MODEM_RESPONSE_CODE_OK )
    {
      if (event_fields.event_type = LR1110_MODEM_LORAWAN_EVENT_GNSS_SCAN_DONE) {
        Serial.println("GNSS event");
        gnss_scan_done( event_fields.buffer, event_fields.buffer_len ); //put GNSS event result in buffer
      }
      else {
        Serial.println("Event type not expected");    //print error when event is not a GNSS event
      }
      Serial.print ("Number of GNSS results: ");
      Serial.println(gnss.capture_result.nb_detected_satellites);
      for (uint8_t i = 0; i < gnss.capture_result.nb_detected_satellites; i++) {
        Serial.print ("GNSS id: ");
        Serial.print (gnss.capture_result.detected_satellites[i].satellite_id);
        Serial.print (" GNSS cnr: ");
        Serial.println (gnss.capture_result.detected_satellites[i].cnr);
      }
      Serial.print ("Number of GNSS nav bytes: ");
      Serial.println(gnss.capture_result.result_size);
      Serial.print ("GNSS nav message: ");
      for ( uint8_t i = 0; i < gnss.capture_result.result_size; i++ ) {
        Serial.print (gnss.capture_result.result_buffer[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      Serial.println("GNSS scan end");
    }
    else
    {
      Serial.print("ERROR: ");
      Serial.println(modem_response_code, HEX);
    }
  }
}
void gnss_scan_done( uint8_t* buffer, uint16_t size ) { //function that copies gnss result in gnss buffer
  memcpy( gnss.capture_result.result_buffer, buffer, size );
  gnss.capture_result.result_size = size;
}
