/*
   LoRa mode A device.
   Support for configuration via downlink .
   Interval with WiFi scan, sensor values and GNSS scan. Send in 2 payloads.
   Interval for control message.
   Information on serial monitor.
   Radio beacon implemented.
   Motion detection interrupt implemented.
   v1.3
*/

#include <skylabLR.h>
#include <ArduinoLowPower.h>
#include <RTCZero.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_LSM303_Accel.h>

#define USB_DEBUG 0 //keep USB serial debug running and prevent sleep when set to 1

/* Set adresses. dev-eui is ignored by default and set by de factory default key, displayed in serial monitor. Pin not used by default*/
uint8_t dev_eui[8]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; //dev_eui
uint8_t join_eui[8]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; //app_eui
uint8_t app_key[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; //app_key
uint32_t pin;
uint16_t intervalTime = 5; //default interval, in minutes
uint16_t motionIntervalTime = 1; //interval when motion detected and motionActivation is on.
uint16_t controlTime = 1415; // default control interval, in minutes, control message in a bit less than every 24h
bool ledActivation = 1; //activation for activity LED

bool WiFiActivation = 1;  //activation for WiFi scan on port 2, also contains the sensor data
bool GNSSActivation = 1;  //activation for GNSS scan on port 3

bool motionActivation = 0;  //activation for motionfrequency

uint16_t timeInMotionInterval = 3; //time to use motion interval since last motion detection, after this amount of minutes the system goes back to the normal intervalTime

#define MAX_GNSS_SCANS_PER_INTERVAL 5 //max retries when not enough sats are found
#define MIN_GNSS_SAT_NEEDED 5 //currently a min of 5 is needed
#define MAX_GNSS_SAT_NEEDED 6 //max sats to scan for
#define MINUTES_AFTER_FAILED_JOIN_TO_REJOIN 60 //minutes to sleep between join retries

/* General LoRaWAN settings*/
#define LORAWAN_DEFAULT_DATARATE LR1110_MODEM_ADR_PROFILE_MOBILE_LOW_POWER //LR1110_MODEM_ADR_PROFILE_NETWORK_SERVER_CONTROLLED | LR1110_MODEM_ADR_PROFILE_MOBILE_LONG_RANGE | LR1110_MODEM_ADR_PROFILE_MOBILE_LOW_POWER | LR1110_MODEM_ADR_PROFILE_CUSTOM
uint8_t adr_custom_list[16] = { 0x05, 0x05, 0x05, 0x04, 0x04, 0x04, 0x03, 0x03, 0x03, 0x02, 0x02, 0x02, 0x01, 0x01, 0x00, 0x00}; //only used when LORAWAN_DEFAULT_DATARATE is set to LR1110_MODEM_ADR_PROFILE_CUSTOM
#define LORAWAN_CONFIRMED_MSG_ON false  //ack messages for uplinks
#define LORAWAN_DEFAULT_REGION LR1110_LORAWAN_REGION_EU868 //LR1110_LORAWAN_REGION_EU868 | LR1110_LORAWAN_REGION_US915
#define LORAWAN_DEFAULT_RF_OUTPUT LR1110_MODEM_RADIO_PA_SEL_HP //LR1110_MODEM_RADIO_PA_SEL_LP | LR1110_MODEM_RADIO_PA_SEL_HP | LR1110_MODEM_RADIO_PA_SEL_LP_HP_LF

Adafruit_BME280 bme;
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

bool beaconActivation = 0;
uint16_t beaconTime = 10;

/*interrupt variables*/
uint32_t counter = 0;
uint32_t counter2 = 0;
uint32_t counter3 = 0;
uint16_t intervalCurrent = intervalTime;
uint16_t motionCountDown = 0;
bool motionActivated = 0;
bool joiningFailed = 0;
uint8_t joiningFailedCount = 0;

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
  if (USB_DEBUG != 1) {
    delay(30000); //start after 30 sec for allowing code to upload before disabling USB.
  }
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
  BME280_init();
  Wire.begin();
  LSM303_init();
  Wire.end();
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
  if (motionActivation == 1) {
    pinMode (INT1 , INPUT);
    LowPower.attachInterruptWakeup(INT1, motion_detect, RISING);
  }
  rtc.begin();
  rtc.attachInterrupt(oneMinute);
  rtc.setSeconds(0);
  rtc.setAlarmSeconds(5);
  rtc.enableAlarm(rtc.MATCH_SS);

}
void loop() {//puts uC in sleepmode when nothing to do
  if (USB_DEBUG != 1) {
    USBDevice.detach();
    LowPower.deepSleep();
  }
}
void beacon_function() {  //function that activates beacon
  if (ledActivation == 1) {
    digitalWrite(snifLED2, LOW);
  } //turn led on when led activation is set to 1
  lr1110_modem_response_code_t modem_response_code;
  for (int i = 0; i < 5; i++){
  modem_response_code = lr1110_modem_test_tx_single(869800000, 22, LR1110_MODEM_TST_MODE_FSK, LR1110_MODEM_TST_MODE_125_KHZ, LR1110_MODEM_TST_MODE_4_5, 255);
  }
  Serial.print("Beacon packet: ");
  Serial.println(modem_response_code);
  counter3++;
  Serial.print("Beacon timer: ");
  Serial.print(counter3);
  Serial.println(" secondes");
  rtc.setSeconds(5);
  digitalWrite(snifLED2, HIGH); //turn the led off
  if (counter3 >= beaconTime * 60) {
    rtc.detachInterrupt();
    lr1110_modem_response_code_t modem_response_code;
    modem_response_code = lr1110_modem_test_exit();
    Serial.print("RADIOBEACON EXIT: ");
    Serial.println(modem_response_code);
    delay(100);
    LR1110_rejoin();
    beaconActivation = 0;
    counter3 = 0;
    rtc.setSeconds(6);
    rtc.attachInterrupt(oneMinute);
  }
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
    counter2 = (controlTime); //retry next minute
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
    counter = (intervalCurrent); //retry next minute
  }
  else {
    if (WiFiActivation == 1) {
      if (ledActivation == 1) {
        digitalWrite(snifLED1, LOW);
      } //turn led on when led activation is set to 1
      LR1110_WiFi_scan();   //scan for WiFi adresses
      digitalWrite(snifLED1, HIGH); //turn the led off
      sendResp = send_payload(WIFI_PAYLOAD_FORMAT);  //send the WiFi payload to the lora network
    }

    if (GNSSActivation == 1) {
      if (ledActivation == 1) {
        digitalWrite(snifLED, LOW);
      } //turn led on when led activation is set to 1
      digitalWrite(LNA, HIGH);  //Turn on the Low Noise Amp for the GNSS antenna
      LR1110_GNSS_scan();     //scan for GNSS data
      digitalWrite(LNA, LOW);   //Turn off the Low Noise Amp for the GNSS antenna
      digitalWrite(snifLED, HIGH); //turn the led off
      sendResp = send_payload(GNSS_PAYLOAD_FORMAT);  //send the GNSS payload to the lora network
    }

    if (WiFiActivation == 0 && GNSSActivation == 0) {
      control_function();
    }
    counter = 1;  //reset the counter
  }
}

void oneMinute () {
  Serial.println("One minute int");
  if (joiningFailed == 1)
  {
    if (joiningFailedCount == 0){
      Serial.print("Joining failed, retry in ");
      Serial.print(MINUTES_AFTER_FAILED_JOIN_TO_REJOIN);
      Serial.println(" minutes");
      lr1110_modem_leave_network();
    }   
    joiningFailedCount++;
    if (joiningFailedCount >= (MINUTES_AFTER_FAILED_JOIN_TO_REJOIN+1))
    {
      Serial.println("Retry to join");
      lr1110_modem_join();
      joiningFailedCount = 0;
      joiningFailed == 0;
    }
  }
  else {
    if (motionActivated == 1) {
      if (motionCountDown <= 0) {
        Serial.println("Back to default interval");
        intervalCurrent = intervalTime;
        motionActivated = 0;
      }
      if (motionCountDown > 0) {
        motionCountDown--;
        Serial.print("Motion interval: ");
        Serial.println(motionCountDown);
      }
    }
    else {}

    if (beaconActivation == 1) {
      if (counter3 == 0) {
        rtc.detachInterrupt();

        lr1110_modem_leave_network();
        lr1110_modem_response_code_t modem_response_code;
        modem_response_code = lr1110_modem_test_mode_start();
        Serial.print("RADIOBEACON START: ");
        Serial.println(modem_response_code);
        rtc.attachInterrupt(beacon_function);
        rtc.setSeconds(5);
      }
    }
    else {
      if (counter >= intervalCurrent) {
        main_function(); //execute the main function
      }
      else {
        counter++;  //count up the counter
      }

      if (counter2 >= controlTime) {
        control_function();
      }
      else {
        counter2++; //count up the counter
      }
    }
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
    uint16_t batteryValue = read_battery();
    tx_frame_buffer[21] = (batteryValue);
    Wire.begin();
    bme.takeForcedMeasurement();
    float BMEtemp = bme.readTemperature();
    int16_t BMEtempS = BMEtemp * 100;
    Serial.print("Temperature: ");
    Serial.println(BMEtemp);
    float BMEpress = bme.readPressure() / 100.0F;
    int16_t BMEpressS = BMEpress * 10;
    Serial.print("Pressure: ");
    Serial.println(BMEpress);
    int8_t BMEhum = bme.readHumidity();
    Serial.print("Humidity: ");
    Serial.println(BMEhum);
    Wire.end();

    tx_frame_buffer[22] = ((BMEtempS) >> 8);
    tx_frame_buffer[23] = ((BMEtempS));
    tx_frame_buffer[24] = ((BMEpressS) >> 8);
    tx_frame_buffer[25] = ((BMEpressS));
    tx_frame_buffer[26] = (BMEhum);
    tx_frame_buffer[27] = motionCountDown /*motionActivated*/; //send motion countdown to confirm the device in in motion interval
    
    modem_response_code = lr1110_modem_request_tx(2, is_tx_confirmed, tx_frame_buffer, 28); //send over port 2, ack defined above, send the buffer, 27 bytes
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
      if (event_fields.buffer[0] == 0) {  //byte 0 contains ledActivation, set ledActivation variable accordingly, or do noting when value is not 0 or 1
        ledActivation = 0;
      }
      if (event_fields.buffer[0] == 1) {
        ledActivation = 1;
      }
      else {}
      uint16_t intervalTimeRec = (event_fields.buffer[1] << 8 | event_fields.buffer[2]);  //read byte 1 and 2 for interval time
      if (intervalTimeRec > 0) {   //only configure new interval time when received time is higher than 0 minutes
        intervalTime = intervalTimeRec;
        intervalCurrent = intervalTime;
      }
      uint16_t beaconTimeRec = (event_fields.buffer[3]);  //read byte 4 and 5 for beacon time
      if (beaconTimeRec > 0) {   //only configure new interval time when received time is higher than 0 minutes
        beaconTime = beaconTimeRec;
        beaconActivation = 1;
      }
      if (event_fields.buffer[4] == 0) {  //byte 0 contains WiFiActivation, set WiFiACtivation variable accordingly, or do noting when value is not 0 or 1
        WiFiActivation = 0;
      }
      if (event_fields.buffer[4] == 1) {
        WiFiActivation = 1;
      }
      if (event_fields.buffer[5] == 0) {  //byte 0 contains GNSSActivation, set GNSSActivation variable accordingly, or do noting when value is not 0 or 1
        GNSSActivation = 0;
      }
      if (event_fields.buffer[5] == 1) {
        GNSSActivation = 1;
      }
      Serial.print("LED activation: ");
      Serial.println(ledActivation);
      Serial.print("Interval: ");
      Serial.println(intervalTime);
      Serial.print("Beacon: ");
      Serial.println(beaconActivation);
      Serial.print("Beacon time: ");
      Serial.println(beaconTime);
      Serial.print("WiFi activation: ");
      Serial.println(WiFiActivation);
      Serial.print("GNSS activation: ");
      Serial.println(GNSSActivation);
    }
   if (event_fields.event_type == LR1110_MODEM_LORAWAN_EVENT_JOIN_FAIL) {
      Serial.println("Join fail event");
      joiningFailed = 1;
      oneMinute();
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
void wifi_scan_done( uint8_t* buffer, uint16_t size ) { //function that puts wifi result in wifi buffer
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
  gnss.settings.constellation_to_use = LR1110_MODEM_GNSS_GPS_MASK /*| LR1110_MODEM_GNSS_BEIDOU_MASK*/;
  gnss.settings.scan_type            = AUTONOMOUS_MODE;
  gnss.settings.search_mode          = LR1110_MODEM_GNSS_OPTION_BEST_EFFORT;
  gnss.settings.input_paramaters     = LR1110_MODEM_GNSS_BIT_CHANGE_MASK | LR1110_MODEM_GNSS_DOPPLER_MASK | LR1110_MODEM_GNSS_PSEUDO_RANGE_MASK;
  lr1110_modem_gnss_set_constellations_to_use(LR1110_MODEM_GNSS_GPS_MASK /*| LR1110_MODEM_GNSS_BEIDOU_MASK*/);    //set the GNSS settings
  gnss.settings.nb_sat = MAX_GNSS_SAT_NEEDED; //max satellites to return, when 0 all detected sattellites are returned
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
  for (uint8_t k = 1; k < MAX_GNSS_SCANS_PER_INTERVAL; k++) {
    if (numberSat < MIN_GNSS_SAT_NEEDED) {
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
  }
}

void gnss_scan_done( uint8_t* buffer, uint16_t size ) { //function that copies gnss result in gnss buffer
  memcpy( gnss.capture_result.result_buffer, buffer, size );
  gnss.capture_result.result_size = size;
}

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
  modem_response_code = lr1110_modem_set_rf_output(LORAWAN_DEFAULT_RF_OUTPUT); //set the rf output
  Serial.println("SET DIO RF SWITCH");
  modem_response_code = lr1110_modem_set_class(LR1110_LORAWAN_CLASS_A); //set the lorawan class to A (only receive downlink after a recent uplink)
  Serial.println("SET CLASS");
  modem_response_code = lr1110_modem_set_region(LORAWAN_DEFAULT_REGION); //set region
  Serial.println("SET REGION");
  modem_response_code = lr1110_modem_activate_duty_cycle(LORAWAN_DUTYCYCLE_ON); //set lorawan dutycycle rules
  Serial.println("SET DUTYCYCLE");
  modem_response_code = lr1110_modem_set_adr_profile(LORAWAN_DEFAULT_DATARATE, adr_custom_list);  //set detarate
  Serial.println("SET ADR PROFILE");
  modem_response_code = lr1110_modem_set_dm_port(1);  //set the port for the dm filed message
  Serial.println("SET DM PORT");
  lr1110_modem_dm_info_fields_t dm_info_fields;
  dm_info_fields.dm_info_field[0] = LR1110_MODEM_DM_INFO_TYPE_UPTIME ;
  dm_info_fields.dm_info_length   = 1;
  modem_response_code = lr1110_modem_set_dm_info_field(&dm_info_fields); //set dm field information
  Serial.println("SET DM INFO FIELD");
  modem_response_code = lr1110_modem_set_dm_info_interval(LR1110_MODEM_REPORTING_INTERVAL_IN_HOUR, 24);  //set the interval of the dm field message
  Serial.println("SET DM INFO INTERVAL");
  modem_response_code = lr1110_modem_set_connection_timeout(255, 2400); //set connection timeout
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
  rf_switch_cfg.enable  = LR1110_MODEM_SYSTEM_RFSW0_HIGH | LR1110_MODEM_SYSTEM_RFSW1_HIGH /*| LR1110_MODEM_SYSTEM_RFSW2_HIGH*/;
  rf_switch_cfg.standby = 0;
  /* LoRa SPDT */
  rf_switch_cfg.rx = LR1110_MODEM_SYSTEM_RFSW0_HIGH;
  rf_switch_cfg.tx = LR1110_MODEM_SYSTEM_RFSW0_HIGH | LR1110_MODEM_SYSTEM_RFSW1_HIGH;
  rf_switch_cfg.tx_hp = LR1110_MODEM_SYSTEM_RFSW1_HIGH;
  /* GNSS ON */
  //rf_switch_cfg.gnss =  LR1110_MODEM_SYSTEM_RFSW2_HIGH;
  rf_switch_cfg.gnss =  0;
  rf_switch_cfg.wifi = 0;
  modem_response_code = lr1110_modem_system_set_dio_as_rf_switch(&rf_switch_cfg); //set the radio switch IO pins on the LR1110
  Serial.println("SET TX POWER OFFSET");
  modem_response_code = lr1110_modem_set_tx_power_offset (0); //set the tx power offset
  Serial.println("SET RF OUTPUT");
  modem_response_code = lr1110_modem_set_rf_output(LORAWAN_DEFAULT_RF_OUTPUT); //set the rf output
  Serial.println("SET DIO RF SWITCH");
  modem_response_code = lr1110_modem_set_class(LR1110_LORAWAN_CLASS_A); //set the lorawan class to A (only receive downlink after a recent uplink)
  Serial.println("SET CLASS");
  modem_response_code = lr1110_modem_set_region(LORAWAN_DEFAULT_REGION); //set region
  Serial.println("SET REGION");
  modem_response_code = lr1110_modem_activate_duty_cycle(LORAWAN_DUTYCYCLE_ON); //set lorawan dutycycle rules
  Serial.println("SET DUTYCYCLE");
  modem_response_code = lr1110_modem_set_adr_profile(LORAWAN_DEFAULT_DATARATE, adr_custom_list);  //set adaptive datarate profile
  Serial.println("SET ADR PROFILE");
  modem_response_code = lr1110_modem_set_dm_port(1);  //set the port for the dm messages
  Serial.println("SET DM PORT");
  lr1110_modem_dm_info_fields_t dm_info_fields;
  dm_info_fields.dm_info_field[0] = LR1110_MODEM_DM_INFO_TYPE_UPTIME ;
  dm_info_fields.dm_info_length   = 1;
  modem_response_code = lr1110_modem_set_dm_info_field(&dm_info_fields); //set dm field information
  Serial.println("SET DM INFO FIELD");
  modem_response_code = lr1110_modem_set_dm_info_interval(LR1110_MODEM_REPORTING_INTERVAL_IN_HOUR, 24);  //set the interval of the dm field message
  Serial.println("SET DM INFO INTERVAL");
  modem_response_code = lr1110_modem_set_connection_timeout(255, 2400); //set connection timeout for receiving downlinks
  Serial.println("SET CONN TIMEOUT");
}

void BME280_init() { //start BME280 sensor
  unsigned status;
  Wire.begin();
  status = bme.begin(0x77); //adress BME280 onboard
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF   );
  Wire.end();
}

void LSM303_init() { //start LSM303 sensor
    Wire.beginTransmission(byte(0b0011001)); //set accelerometer sensor
    Wire.write(byte(0x24));
    Wire.write(byte(0b10000000)); //interrupt enable reg3
    Wire.endTransmission();
    if (motionActivation == 1) {
    accel.begin();
    accel.setRange(LSM303_RANGE_4G);
    accel.setMode(LSM303_MODE_LOW_POWER);
    Wire.beginTransmission(byte(0b0011001)); //set accelerometer sensor
    Wire.write(byte(0x21));
    Wire.write(byte(0b00000001)); //interrupt config
    Wire.endTransmission();

    Wire.beginTransmission(byte(0b0011001)); //set accelerometer sensor
    Wire.write(byte(0x25));
    Wire.write(byte(0b1000001)); //interrupt config
    Wire.endTransmission();

    Wire.beginTransmission(byte(0b0011001)); //set accelerometer sensor
    Wire.write(byte(0x30));
    Wire.write(byte(0b11010101)); //interrupt config
    Wire.endTransmission();

    Wire.beginTransmission(byte(0b0011001)); //set accelerometer sensor
    Wire.write(byte(0x32));
    Wire.write(byte(0b00000010)); //interrupt threshold value
    Wire.endTransmission();

    Wire.beginTransmission(byte(0b0011001)); //set accelerometer sensor
    Wire.write(byte(0x33));
    Wire.write(byte(0b00000011)); //interrupt duration
    Wire.endTransmission();

    Wire.beginTransmission(byte(0b0011001)); //set accelerometer sensor
    Wire.write(byte(0x22));
    Wire.write(byte(0b11000000)); //interrupt enable reg3
    Wire.endTransmission();
  }
  if (motionActivation == 0) {
    Wire.beginTransmission(byte(0b0011001)); //set accelerometer sensor to idle
    Wire.write(byte(0x20));
    Wire.write(byte(0b00001000));
    Wire.endTransmission();
  }

  Wire.beginTransmission(byte(0b0011110));  //set magneto sensor to idle
  Wire.write(byte(0x60));
  Wire.write(byte(0b01110010));
  Wire.endTransmission();

  Wire.beginTransmission(byte(0b0011110));  //set magneto sensor to idle
  Wire.write(byte(0x61));
  Wire.write(byte(0b0001010));
  Wire.endTransmission();
}
void motion_detect() {
  if (beaconActivation != 1 && joiningFailed != 1) {
    if (motionActivated == 0) {
      motionActivated = 1;
      rtc.setSeconds(5);
    }
    intervalCurrent = motionIntervalTime;
    motionCountDown = timeInMotionInterval;
    Serial.println("Motion detected");
  }
}
