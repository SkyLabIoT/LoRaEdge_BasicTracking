/*
   LoRa mode A device.
   Support for configuration via downlink .
   Interval with WiFi scan, sensor values and GNSS scan. Send in 2 payloads.
   Interval for control message.
   Information on serial monitor.
   Radio beacon implemented.
   Motion detection interrupt implemented.
   v1.5.1
*/

#include <skylabLR.h>
#include <Wire.h>
#include <ArduinoLowPower.h> //Arduino SAMD21 Low Power library https://www.arduino.cc/en/Reference/ArduinoLowPower
#include <RTCZero.h> //Arduino RTC library https://www.arduino.cc/en/Reference/RTC
#include <Adafruit_BME280.h> //Adafruit BME280 library https://github.com/adafruit/Adafruit_BME280_Library
#include <Adafruit_LSM303_Accel.h> //Adafruit LMS303 accelerometer library https://github.com/adafruit/Adafruit_LSM303_Accel
#include <FlashAsEEPROM.h> //FlashStorage library by cmaglie https://github.com/cmaglie/FlashStorage

#define USB_DEBUG 0 //keep USB serial debug running and prevent sleep when set to 1

/* Set adresses. dev-eui is ignored by default and set by de factory default key, displayed in serial monitor. Pin not used by default*/
uint8_t dev_eui[8]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; //dev_eui
uint8_t join_eui[8]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; //app_eui
uint8_t app_key[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; //app_key
uint32_t pin; //pin

/*Activation options*/
bool ledActivation = 1;   //activation for activity LED
bool WiFiActivation = 1;  //activation for WiFi scan on port 2, also contains the sensor data
bool GNSSActivation = 1;  //activation for GNSS scan on port 3
bool motionActivation = 1;  //activation for motion detection interrupt
bool alcSyncEnable = 0; //enable alc sync service on port 202 to sync time. not yet used for anything. see: https://lora-alliance.org/resource-hub/lorawanr-application-layer-clock-synchronization-specification-v100

/*Interval options*/
uint16_t intervalTime = 15; //default interval, in minutes
uint16_t controlTime = 1415; // default control interval, in minutes, control message in a bit less than every 24h
uint16_t motionIntervalTime = 1; //interval when motion detected and motionActivation is on.
uint16_t timeInMotionInterval = 5; //time to use motion interval since last motion detection, after this amount of minutes the system goes back to the normal intervalTime

/*GNSS scan options*/
#define MAX_GNSS_SCANS_PER_INTERVAL 5 //max retries when not enough sats are found during GNSS scan
#define MIN_GNSS_SAT_NEEDED 5 //minimum amound of sats needded, currently a min of 5 is needed, if minimum amount of sats is found no retry will be performed
#define MAX_GNSS_SAT_NEEDED 6 //max sats to scan for, to limit GNSS payload
#define MINUTES_AFTER_FAILED_JOIN_TO_REJOIN intervalTime //minutes to sleep if join attemped failed, after this amount of minutes a join retry is attempted

/*Motion detection registers*/
byte acc_threshold = 0b00000111; //motion detection threshold (LSM303AGR accelerometer threshold register)
byte acc_duration = 0b00000010; //motion detection duration (LSM303AGR accelerometer duration register)

/* General LoRaWAN settings*/
#define LORAWAN_DEFAULT_DATARATE LR1110_MODEM_ADR_PROFILE_MOBILE_LOW_POWER //adaptive datarate options are: LR1110_MODEM_ADR_PROFILE_NETWORK_SERVER_CONTROLLED | LR1110_MODEM_ADR_PROFILE_MOBILE_LONG_RANGE | LR1110_MODEM_ADR_PROFILE_MOBILE_LOW_POWER | LR1110_MODEM_ADR_PROFILE_CUSTOM
uint8_t adr_custom_list[16] = { 0x05, 0x05, 0x05, 0x04, 0x04, 0x04, 0x03, 0x03, 0x03, 0x02, 0x02, 0x02, 0x01, 0x01, 0x00, 0x00 }; //only used when LORAWAN_DEFAULT_DATARATE is set to LR1110_MODEM_ADR_PROFILE_CUSTOM
#define LORAWAN_CONFIRMED_MSG_ON false  //ack messages for uplinks
#define LORAWAN_DEFAULT_REGION LR1110_LORAWAN_REGION_EU868 //region options are: LR1110_LORAWAN_REGION_EU868 | LR1110_LORAWAN_REGION_US915
#define LORAWAN_DEFAULT_RF_OUTPUT LR1110_MODEM_RADIO_PA_SEL_HP //LoRa antenna power options are: LR1110_MODEM_RADIO_PA_SEL_LP | LR1110_MODEM_RADIO_PA_SEL_HP | LR1110_MODEM_RADIO_PA_SEL_LP_HP_LF

/*Non user definable variables and settings*/
Adafruit_BME280 bme;  //init BME280 lib
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321); //init LSM303 accelerometer lib
bool beaconActivation = 0; //variable to
uint16_t beaconTime = 10;
uint32_t counter = intervalTime;
uint32_t counter2 = 0;
uint32_t counter3 = 0;
uint16_t intervalCurrent = intervalTime;
uint16_t motionCountDown = 0;
bool motionActivated = 0;
bool joiningFailed = 1;
uint8_t joiningFailedCount = 0;
#define WIFI_MAX_BASIC_RESULTS_PER_SCAN 3 //Max adresses to scan for when wifi sniffing
#define WIFI_PAYLOAD_FORMAT 2 //define payload type for Wi-Fi send function
#define GNSS_PAYLOAD_FORMAT 3 //define payload type for GNSS send function
#define CONFIG_PAYLOAD_FORMAT 4 //define payload type for configuration send function
static lr1110_modem_uplink_type_t is_tx_confirmed = (lr1110_modem_uplink_type_t) LORAWAN_CONFIRMED_MSG_ON;
static uint8_t tx_frame_buffer[LORAWAN_APP_DATA_MAX_SIZE];
wifi_t wifi;
gnss_t gnss;
lr1110_modem_event_fields_t  event_fields;
lr1110_modem_event_t* lr1110_modem_event;
RTCZero rtc;  //init rtc lib

void setup() {
  /*Set serial monitor*/
  if (USB_DEBUG != 1) {
    delay(30000); //start after 30 sec for allowing code to upload before disabling USB.
  }
  Serial.begin(9600); //start serial monitor on 9600 baud rate
  Serial.println();
  Serial.println("BEGIN");
  /*Set pinmodes*/
  pinMode (NSS, OUTPUT);
  pinMode (LEDG, OUTPUT);
  pinMode (LEDB, OUTPUT);
  pinMode (LEDR, OUTPUT);
  pinMode (busyPin, INPUT);
  pinMode (RESET, OUTPUT);
  pinMode (LNA, OUTPUT);
  pinMode (eventPin, INPUT);
  pinMode (batPin, INPUT);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
  digitalWrite(LEDR, HIGH);
  /*Start SPI*/
  SPI.begin();
  /*Init sensors*/
  BME280_init();
  LSM303_init();
  /*Init the LR1110 transceiver*/
  LR1110_init();
  /*Set keys*/
  Serial.println("LR KEYS");
  delay(100);
  Serial.print("Dev eui: ");
  lr1110_modem_response_code_t modem_response_code;
  modem_response_code = lr1110_modem_get_dev_eui(dev_eui); //request the dev eui from the LR1110 and print in on the serial monitor
  for (uint8_t i = 0; i < 8; i++) {
    if(dev_eui[i] < 0x10) Serial.print("0");
    Serial.print(dev_eui[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  Serial.print("Join eui: ");
  modem_response_code = lr1110_modem_set_join_eui(join_eui); //set the join eui and print in on the serial monitor
  for (uint8_t i = 0; i < 8; i++) {
    if(join_eui[i] < 0x10) Serial.print("0");
    Serial.print(join_eui[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  Serial.print("App key: ");
  modem_response_code = lr1110_modem_set_app_key(app_key); //set the app key and print in on the serial monitor
  for (uint8_t i = 0; i < 16; i++) {
    if(app_key[i] < 0x10) Serial.print("0");
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
  Serial.print( "LORAWAN     : 0x" );
  Serial.println(modem.lorawan, HEX);
  Serial.print( "FIRMWARE    : 0x" );
  Serial.println( modem.firmware, HEX);
  Serial.print( "BOOTLOADER  : 0x");
  Serial.println(modem.bootloader, HEX);
  delay(10);
  /*Init EEPROM values*/
  read_config_flashEEPROM();
  /*Join network*/
  LR1110_join();
  if (ledActivation == 1) {
    digitalWrite(LEDB, LOW);
    digitalWrite(LEDR, LOW);
  } //turn led on when led activation is set to 1
  /*Check for events for first time*/
  LR1110_event_flush();
  /*Set pin interrupt settings*/
  LowPower.attachInterruptWakeup(eventPin, LR1110_event_flush, RISING); //attach the event function to the LR1110 event pin
  if (motionActivation == 1) {
    pinMode (INT1 , INPUT);
    LowPower.attachInterruptWakeup(INT1, motion_detect, RISING);  //attach motion detection function to LSM303 interrupt pin 1
  }
  /*Configure RTC for timed interrupt*/
  rtc.begin();
  rtc.attachInterrupt(one_minute); //attach the one minute function to the RTC timer interrupt
  rtc.setSeconds(10); //start clock at 10 seconds
  rtc.setAlarmSeconds(5); //set interrupt alarm at 5 seconds
  rtc.enableAlarm(rtc.MATCH_SS);  //enable interrupt on second match

}
void loop() { //puts uC in sleepmode when nothing to do and USB debug is disabled
  if (USB_DEBUG != 1) {
    USBDevice.detach(); //detach USB device
    LowPower.deepSleep(); //enter deep sleep
  }
}

void beacon_function() {  //function that activates beacon
  if (ledActivation == 1) {
    digitalWrite(LEDG, LOW);
  } //turn led on when led activation is set to 1
  lr1110_modem_response_code_t modem_response_code;
  for (int i = 0; i < 5; i++) {
    modem_response_code = lr1110_modem_test_tx_single(869800000, 22, LR1110_MODEM_TST_MODE_FSK, LR1110_MODEM_TST_MODE_125_KHZ, LR1110_MODEM_TST_MODE_4_5, 255);
  } //send 5 random LoRa payloads on 869800000 Hz, -22dbm, FSK modulation, 125kHz bandwidth, 255 bytes per payload
  Serial.print("Beacon packet: ");
  Serial.println(modem_response_code); //print command response from last packet
  counter3++; //add 1 to counter 3
  Serial.print("Beacon timer: ");
  Serial.print(counter3); // print counter 3
  Serial.println(" secondes");
  rtc.setSeconds(5);  //set time to 5 second so that interrupt occurs again next second
  digitalWrite(LEDG, HIGH); //turn the led off
  if (counter3 >= beaconTime * 60) {  //if counter reaches the set beacon time
    rtc.detachInterrupt();  //detach the one minute interrupt
    lr1110_modem_response_code_t modem_response_code;
    modem_response_code = lr1110_modem_test_exit(); //exit test mode
    Serial.print("RADIOBEACON EXIT: ");
    Serial.println(modem_response_code);
    delay(100);
    LR1110_rejoin();  //rejoin network
    beaconActivation = 0; //turn beacon off
    counter3 = 0; //reset counter 3
    rtc.setSeconds(6);  //set RTC time to 6 seconds so that next minute interrupt is in 1 minute
    rtc.attachInterrupt(one_minute);  //reattach one minute interrupt to RTC alarm
  }
}
void control_function() { //function for sending the control message
  lr1110_modem_status_t modemStat;
  lr1110_modem_get_status(&modemStat);
  Serial.print("Modem status: ");
  Serial.println(modemStat);
  if (modemStat != 0x08)  //check for a joined modem status
  {
    lr1110_modem_response_code_t modem_response_code;
    modem_response_code = lr1110_modem_leave_network(); //if status is not joined, completly leave network
    Serial.println("Modem ERROR, leaving network and try to rejoin");
    Serial.print("Leaving network response: ");
    Serial.println(modem_response_code);
    LR1110_rejoin();  //rejoin network
    counter2 = (controlTime); //retry next minute
  }
  else {
    static uint8_t tx_frame_buffer_control[LORAWAN_APP_DATA_MAX_SIZE];
    uint8_t batteryValue = read_battery();  //read battery voltage
    tx_frame_buffer_control[0] = batteryValue;  //set battery voltage in send buffer
    lr1110_modem_request_tx(10, is_tx_confirmed, tx_frame_buffer_control, 1);  //request modem to send payload on port 10, length of 1 byte
    wait_on_event_pin(20);//wait 20 sec for first event to confirm payload is send
    Serial.println("Control message send");
    counter2 = 1; //reset counter 2 to 1
  }
}
void main_function() {  //function to send the data
  Serial.println();
  Serial.println("...SEND INTERRUPT...");
  lr1110_modem_status_t modemStat;
  lr1110_modem_get_status(&modemStat);
  Serial.print("Modem status: ");
  Serial.println(modemStat);
   if (modemStat != 0x08)  //check for a joined modem status
  {
    lr1110_modem_response_code_t modem_response_code;
    modem_response_code = lr1110_modem_leave_network(); //if status is not joined, completly leave network
    Serial.println("Modem ERROR, leaving network and try to rejoin");
    Serial.print("Leaving network response: ");
    Serial.println(modem_response_code);
    LR1110_rejoin();  //rejoin network
    counter = (intervalCurrent); //retry next minute
  }
  else {
    if (WiFiActivation == 1) {  //if Wi-Fi payload is activated
      if (ledActivation == 1) {
        digitalWrite(LEDB, LOW);
      } //turn led on when led activation is set to 1
      LR1110_WiFi_scan();   //scan for WiFi adresses
      send_payload(WIFI_PAYLOAD_FORMAT);  //send the WiFi payload to the lora network
      digitalWrite(LEDB, HIGH); //turn the led off
      wait_on_event_pin(20);//wait 20 sec for first event to confirm payload is send
    }
    if (GNSSActivation == 1) {  //if GNSS payload is activated
      if (ledActivation == 1) {
        digitalWrite(LEDR, LOW);
      } //turn led on when led activation is set to 1
      digitalWrite(LNA, HIGH);  //Turn on the Low Noise Amp for the GNSS antenna
      LR1110_GNSS_scan(); //scan for GNSS data
      digitalWrite(LNA, LOW); //Turn off the Low Noise Amp for the GNSS antenna
      send_payload(GNSS_PAYLOAD_FORMAT);  //send the GNSS payload to the lora network
      digitalWrite(LEDR, HIGH); //turn the led off
      wait_on_event_pin(20);  //wait 20 sec for first event to confirm payload is send
    }

    if (WiFiActivation == 0 && GNSSActivation == 0) { //if no payload is activated, fallback to sending control function on main interval
      control_function();
    }
    counter = 1;  //reset the counter
  }
}

void one_minute () {
  Serial.println("One minute int");
  if (joiningFailed == 1) //if joining failed
  {
    if (joiningFailedCount == 0) {  //if first minute after failed join
      Serial.print("Joining failed, retry in ");
      Serial.print(MINUTES_AFTER_FAILED_JOIN_TO_REJOIN);
      Serial.println(" minutes");
      lr1110_modem_leave_network(); //completely leave network
    }
    joiningFailedCount++; //add one to minute counter after failed join event
    if (joiningFailedCount >= (MINUTES_AFTER_FAILED_JOIN_TO_REJOIN + 1))  //if minute counter after failed join event is bigger than configured join retry minutes
    {
      Serial.println("Retry to join");
      joiningFailedCount = 0; //reset minute counter
      lr1110_modem_join();  //rejoin network
      if (ledActivation == 1) {
        digitalWrite(LEDB, LOW);
        digitalWrite(LEDR, LOW);
      } //turn led on when led activation is set to 1
    }
  }
  else {  //if system is joined
    if (motionActivated == 1) { //if device is in motion
      if (motionCountDown > 0) {  //if motion counter is higer than 0
        motionCountDown--;  //count down motion counter
        Serial.print("Motion interval: "); 
        Serial.println(motionCountDown);
      }
      if (motionCountDown <= 0) { //if motion counter is back to 0
        counter = intervalCurrent; //set counter to current interval for last motion payload sending
      }
    }
    else {}

    if (beaconActivation == 1) {  //if beacon is activated
      if (counter3 == 0) {
        rtc.detachInterrupt();  //detach timed interrupts
        lr1110_modem_leave_network(); //leave network
        lr1110_modem_response_code_t modem_response_code;
        modem_response_code = lr1110_modem_test_mode_start(); //set modem to test mode
        Serial.print("RADIOBEACON START: ");
        Serial.println(modem_response_code);
        rtc.attachInterrupt(beacon_function); //attach modem function to RTC timer interrupt
        rtc.setSeconds(5);  //set RTC time to 5 seconds so that interrupt occures next second
      }
    }
    else {  //if beacon is not activated
      if (counter >= intervalCurrent) { //if counter reached the current interval
        Serial.print("Time synced: ");
        uint32_t timeCurrent = 0;
        lr1110_modem_get_gps_time(&timeCurrent); //request synced time
        Serial.println(timeCurrent);
        main_function(); //execute the main function
      }
      else { //if counter did not yet reached the current interval
        counter++;  //add 1 to the counter
      }

      if (counter2 >= controlTime) {  //if counter2 reached the control interval
        control_function(); //execute the control payload function
      }
      else {  //if counter2 did not yet reached the control interval
        counter2++; //count up the counter
      }
    }
   if (motionActivated == 1) { //if device is in motion
      if (motionCountDown <= 0) { //if motion counter is back to 0
        Serial.println("Back to default interval");
        intervalCurrent = intervalTime; //go back to default interval time
        motionActivated = 0;  //reset device to no motion
      }
   }
  }
}
uint8_t send_payload(uint8_t payloadKind) {
  lr1110_modem_response_code_t modem_response_code;
  if (payloadKind == WIFI_PAYLOAD_FORMAT) { //if requested payload type is the Wi-Fi payload
    /*Set Wi-Fi results in buffer and check for amound of Wi-Fi scan results, if not 3 the missing results will be set to 0*/
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
      for (int k = 7; k < 21; k++) {
        tx_frame_buffer[k] = 0;
      }
    }
    else if (wifi.results.nbrResults == 0) {
      for (int k = 0; k < 21; k++) {
        tx_frame_buffer[k] = 0;
      }
    }
    uint16_t batteryValue = read_battery(); //read the battery voltage
    tx_frame_buffer[21] = (batteryValue); //put battery voltage in buffer
    Wire.begin(); //begin i2c communication
    bme.takeForcedMeasurement();  //take BME280 measurements
    float BMEtemp = bme.readTemperature();  //read temperature
    int16_t BMEtempS = BMEtemp * 100; //put temperature from float to int
    Serial.print("Temperature: ");
    Serial.println(BMEtemp);
    float BMEpress = bme.readPressure() / 100.0F; //read pressure
    int16_t BMEpressS = BMEpress * 10; //put pressure from float to int
    Serial.print("Pressure: ");
    Serial.println(BMEpress);
    int8_t BMEhum = bme.readHumidity(); //read humidity
    Serial.print("Humidity: ");
    Serial.println(BMEhum);
    Wire.end(); //end i2c communication
    /*Put sensor data in buffer*/
    tx_frame_buffer[22] = ((BMEtempS) >> 8);
    tx_frame_buffer[23] = ((BMEtempS));
    tx_frame_buffer[24] = ((BMEpressS) >> 8);
    tx_frame_buffer[25] = ((BMEpressS));
    tx_frame_buffer[26] = (BMEhum);
    tx_frame_buffer[27] = motionCountDown; //put motion countdown in buffer to give indication if the device is in motion interval
    modem_response_code = lr1110_modem_request_tx(2, is_tx_confirmed, tx_frame_buffer, 28); //send over port 2, ack defined above, send the buffer, 28 bytes
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
  if (payloadKind == GNSS_PAYLOAD_FORMAT) { //if requested payload type is the GNSS payload
    uint8_t GnssPaySize = 0;
    if (gnss.capture_result.result_size < 100) {  //configure maximum payload byte size
      GnssPaySize = gnss.capture_result.result_size;
    }
    else {
      GnssPaySize = 100;
    }
    for ( uint8_t i = 0; i < GnssPaySize; i++ ) {
      tx_frame_buffer[i] = gnss.capture_result.result_buffer[i];  //put all GNSS data in buffer
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
  if (payloadKind == CONFIG_PAYLOAD_FORMAT) { //if requested payload type is the configuration payload
    /*Put current configuration in buffer*/
    tx_frame_buffer[0] = ledActivation;
    tx_frame_buffer[1] = intervalTime >> 8;
    tx_frame_buffer[2] = intervalTime;
    if (beaconActivation == 0) {
      tx_frame_buffer[3] = 0; //put 0 in buffer when beacon is not activated
    }
    if (beaconActivation != 0) {
      tx_frame_buffer[3] = beaconTime;  //put configured beacon time in buffer when beacon is activated
    }
    tx_frame_buffer[4] = WiFiActivation;
    tx_frame_buffer[5] = GNSSActivation;
    if (motionActivation == 0) {
      tx_frame_buffer[6] = 0; //put 0 in buffer when motion is not activated
    }
    if (motionActivation != 0) {
      tx_frame_buffer[6] = motionIntervalTime;;  //put configured motion time interval in buffer when motion is activated
    }
    tx_frame_buffer[7] = timeInMotionInterval;
    tx_frame_buffer[8] = acc_threshold;
    tx_frame_buffer[9] = acc_duration;
    modem_response_code = lr1110_modem_request_tx(44, is_tx_confirmed, tx_frame_buffer, 10 );//send over port 44, ack defined above, send the buffer, size of 10 bytes
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
void LR1110_event_flush() { //function flushes the open events in the LR1110 and reads the downlink if the event is of the downlink type
  lr1110_modem_response_code_t modem_response_code;
  while ( digitalRead (eventPin) == 1 ) //only do something when eventpin is high and keep doing until pin goes low
  {
    modem_response_code = lr1110_modem_get_event( &event_fields );  //request event from register
    Serial.print("Event type (hex): ");
    Serial.println(event_fields.event_type, HEX);
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
        if (event_fields.buffer[i] < 0x10) Serial.print("0");
        Serial.print (event_fields.buffer[i], HEX);
        Serial.print (" ");
      }
      Serial.println ();
      Serial.print("On port: ");
      Serial.println (port);
      if (port == 2) {
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
        if (event_fields.buffer[6] == 0) {  //if byte 6 is 0 disable motion detection
          motionActivation = 0;
          motionCountDown = 0;
          detachInterrupt(INT1);
        }
        if (event_fields.buffer[6] != 0) {  //if byte is not 0 enable motion detection with the value as interval time
          motionActivation = 1;
          motionIntervalTime = event_fields.buffer[6];
          intervalCurrent = motionIntervalTime;
          pinMode (INT1 , INPUT);
          LowPower.attachInterruptWakeup(INT1, motion_detect, RISING);
        }
        if (event_fields.buffer[7] != 0) {  //if byte 7 is not 0, use byte 7 as time in motion interval, use byte 8 and 9 as values for the LSM303 registers, if byte 7 is 0 ignore the last bytes
          timeInMotionInterval = event_fields.buffer[7];
          acc_threshold = event_fields.buffer[8];
          acc_duration = event_fields.buffer[9];
          motion_detect_set_threshold();
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
        Serial.print("Motion activation: ");
        Serial.println(motionActivation);
        Serial.print("Motion interval:  ");
        Serial.println(motionIntervalTime);
        Serial.print("Motion interval duration: ");
        Serial.println(timeInMotionInterval);
        Serial.print("Motion threshold: ");
        Serial.println(acc_threshold);
        Serial.print("Motion duration: ");
        Serial.println(acc_duration);
        send_payload(CONFIG_PAYLOAD_FORMAT);
        write_config_flashEEPROM();
      }
      if (port == 1) {
        if (event_fields.buffer[0] == 1) {
          if (ledActivation == 1) {
            digitalWrite(LEDB, LOW);
          } //turn led on when led activation is set to 1
          LR1110_WiFi_scan();   //scan for WiFi adresses
          send_payload(WIFI_PAYLOAD_FORMAT);  //send the WiFi payload to the lora network
          digitalWrite(LEDB, HIGH); //turn the led off
          wait_on_event_pin(20);//wait 20 sec for first event to confirm payload is send
          if (ledActivation == 1) {
            digitalWrite(LEDR, LOW);
          } //turn led on when led activation is set to 1
          digitalWrite(LNA, HIGH);  //Turn on the Low Noise Amp for the GNSS antenna
          LR1110_GNSS_scan(); //scan for GNSS data
          digitalWrite(LNA, LOW); //Turn off the Low Noise Amp for the GNSS antenna
          send_payload(GNSS_PAYLOAD_FORMAT);  //send the GNSS payload to the lora network
          digitalWrite(LEDR, HIGH); //turn the led off
          wait_on_event_pin(20);  //wait 20 sec for first event to confirm payload is send
        }
        if (event_fields.buffer[0] == 2) {
          send_payload(CONFIG_PAYLOAD_FORMAT);
          wait_on_event_pin(20);  //wait 20 sec for first event to confirm payload is send
        }
        if (event_fields.buffer[0] == 3) {
          static uint8_t tx_frame_buffer_control[LORAWAN_APP_DATA_MAX_SIZE];
          uint8_t batteryValue = read_battery();  //read battery voltage
          tx_frame_buffer_control[0] = batteryValue;  //set battery voltage in send buffer
          lr1110_modem_request_tx(10, is_tx_confirmed, tx_frame_buffer_control, 1);  //request modem to send payload on port 10, length of 1 byte
          wait_on_event_pin(20);//wait 20 sec for first event to confirm payload is send
          Serial.println("Control message send");
        }
      }
    }
    if (event_fields.event_type == LR1110_MODEM_LORAWAN_EVENT_JOIN_FAIL) {
      Serial.println("Join fail event");
      joiningFailed = 1;  //when join fail event occurs, set this value to 1
      digitalWrite(LEDB, HIGH);
      digitalWrite(LEDR, HIGH);
      one_minute(); //immediately go to one minute function
    }
    if (event_fields.event_type == LR1110_MODEM_LORAWAN_EVENT_TIME_UPDATED_ALC_SYNC) {
      Serial.print("Time synced: ");
      uint32_t timeCurrent = 0;
      lr1110_modem_get_gps_time(&timeCurrent);  //read time from LR1110 after alc sync event
      Serial.println(timeCurrent);
    }
    if (event_fields.event_type == LR1110_MODEM_LORAWAN_EVENT_JOINED) {
      Serial.println("Join succes event");
      digitalWrite(LEDB, HIGH);
      digitalWrite(LEDR, HIGH);
      joiningFailed = 0;  //when join event occurs, set this value to 0
    }
  }
  return;
}
void LR1110_WiFi_scan() { //function to do a Wi-Fi scan
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
void LR1110_GNSS_scan() {//function to do a GNSS scan
  lr1110_modem_response_code_t modem_response_code;
  uint8_t numberSat = 0;
  gnss.settings.enabled              = true;
  gnss.settings.constellation_to_use = LR1110_MODEM_GNSS_GPS_MASK /*| LR1110_MODEM_GNSS_BEIDOU_MASK*/;
  gnss.settings.scan_type            = ASSISTED_MODE;
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
  for (uint8_t k = 1; k < MAX_GNSS_SCANS_PER_INTERVAL; k++) { //retry GNSS scan if not enough sats are found
    if (numberSat < MIN_GNSS_SAT_NEEDED) {  //only do when number of found sats is less then needed
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

void LR1110_rejoin() { //function to reconfigure the LR1110 and rejoin the network
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
  rf_switch_cfg.gnss = 0;
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
  modem_response_code = lr1110_modem_set_dm_port(199);  //set the port for the dm filed message
  Serial.println("SET DM PORT");
  lr1110_modem_dm_info_fields_t dm_info_fields;
  dm_info_fields.dm_info_field[0] = LR1110_MODEM_DM_INFO_TYPE_UPTIME ;
  //dm_info_fields.dm_info_field[1] = LR1110_MODEM_DM_INFO_TYPE_GNSS_ALMANAC_STATUS ;
  dm_info_fields.dm_info_length   = 1; //2
  modem_response_code = lr1110_modem_set_dm_info_field(&dm_info_fields); //set dm field information
  Serial.println("SET DM INFO FIELD");
  modem_response_code = lr1110_modem_set_dm_info_interval(LR1110_MODEM_REPORTING_INTERVAL_IN_HOUR, 24);  //set the interval of the dm field message
  Serial.println("SET DM INFO INTERVAL");
  modem_response_code = lr1110_modem_set_connection_timeout(255, 2400); //set connection timeout
  Serial.println("SET CONN TIMEOUT");
  if (alcSyncEnable == 1) {
    enable_alc_sync();
  }
  LR1110_join();
  if (ledActivation == 1) {
    digitalWrite(LEDB, LOW);
    digitalWrite(LEDR, LOW);
  }
}
void LR1110_init() {  //function for initial initialisation for the LR1110
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
  modem_response_code = lr1110_modem_set_dm_port(199);  //set the port for the dm messages
  Serial.println("SET DM PORT");
  lr1110_modem_dm_info_fields_t dm_info_fields;
  dm_info_fields.dm_info_field[0] = LR1110_MODEM_DM_INFO_TYPE_UPTIME ;
  //dm_info_fields.dm_info_field[1] = LR1110_MODEM_DM_INFO_TYPE_GNSS_ALMANAC_STATUS ;
  dm_info_fields.dm_info_length   = 1;//2
  modem_response_code = lr1110_modem_set_dm_info_field(&dm_info_fields); //set dm field information
  Serial.println("SET DM INFO FIELD");
  modem_response_code = lr1110_modem_set_dm_info_interval(LR1110_MODEM_REPORTING_INTERVAL_IN_HOUR, 24);  //set the interval of the dm field message
  Serial.println("SET DM INFO INTERVAL");
  modem_response_code = lr1110_modem_set_connection_timeout(255, 2400); //set connection timeout
  Serial.println("SET CONN TIMEOUT");
  if (alcSyncEnable == 1) {
    enable_alc_sync();
  }
}

void BME280_init() { //start BME280 sensor
  unsigned status;
  Wire.begin(); //start i2c
  status = bme.begin(0x77); //adress BME280 onboard
  bme.setSampling(Adafruit_BME280::MODE_FORCED, //forced mode
                  Adafruit_BME280::SAMPLING_X1, // temperature sampling
                  Adafruit_BME280::SAMPLING_X1, // pressure sampling
                  Adafruit_BME280::SAMPLING_X1, // humidity sampling
                  Adafruit_BME280::FILTER_OFF   );  //no filter
  Wire.end(); //stop i2c
}
void LSM303_init() { //start LSM303 sensor
  Wire.begin(); //start i2c
  Wire.beginTransmission(byte(0b0011001)); //set accelerometer sensor
  Wire.write(byte(0x24)); //CTRL_REG5_A
  Wire.write(byte(0b10000000)); //reboot memory content
  Wire.endTransmission();
  if (motionActivation == 1) {
    accel.begin();
    accel.setRange(LSM303_RANGE_2G);  //enable 2G range
    accel.setMode(LSM303_MODE_LOW_POWER); //enable low power mode
    Wire.beginTransmission(byte(0b0011001)); //set accelerometer sensor
    Wire.write(byte(0x21)); //CTRL_REG2_A
    Wire.write(byte(0b00000001)); //filter settings
    Wire.endTransmission();

    Wire.beginTransmission(byte(0b0011001)); //set accelerometer sensor
    Wire.write(byte(0x25)); //CTRL_REG6_A
    Wire.write(byte(0b1000001)); //interrupt config, active low
    Wire.endTransmission();

    Wire.beginTransmission(byte(0b0011001)); //set accelerometer sensor
    Wire.write(byte(0x30)); //INT1_CFG_A
    Wire.write(byte(0b11010101)); //6-direction position recognition, enable interrupt on Z, X and Y low events
    Wire.endTransmission();

    Wire.end(); //stop i2c
    motion_detect_set_threshold(); //configure threshold and duration settings
    Wire.begin(); //start i2c

    Wire.beginTransmission(byte(0b0011001)); //set accelerometer sensor
    Wire.write(byte(0x22)); //CTRL_REG3_A 
    Wire.write(byte(0b11000000)); //interrupt1 enable
    Wire.endTransmission();
  }
  if (motionActivation == 0) {
    Wire.beginTransmission(byte(0b0011001)); //set accelerometer sensor to idle
    Wire.write(byte(0x20)); //CTRL_REG1_A
    Wire.write(byte(0b00001000)); //enable low power mode
    Wire.endTransmission();
  }

  Wire.beginTransmission(byte(0b0011110));  //magneto sensor adress
  Wire.write(byte(0x60)); //CFG_REG_A_M
  Wire.write(byte(0b01110010)); //put sensor in lowest power mode / idle
  Wire.endTransmission();

  Wire.beginTransmission(byte(0b0011110));  ///magneto sensor adress
  Wire.write(byte(0x61)); //CFG_REG_B_M 
  Wire.write(byte(0b0001010));  //put sensor in lowest power mode / idle
  Wire.endTransmission();
  Wire.end();
}
void motion_detect_set_threshold() {
  Wire.begin(); //start ic2
  Wire.beginTransmission(byte(0b0011001)); //set accelerometer sensor
  Wire.write(byte(0x32)); //INT1_THS_A
  Wire.write(byte(acc_threshold)); //interrupt threshold value
  Wire.endTransmission();

  Wire.beginTransmission(byte(0b0011001)); //set accelerometer sensor
  Wire.write(byte(0x33)); //INT1_DURATION_A
  Wire.write(byte(acc_duration)); //interrupt duration
  Wire.endTransmission();
  Wire.end(); //stop i2c
}
void motion_detect() {
  if (beaconActivation != 1 && joiningFailed != 1) {  //only do things when beacon is not activated and joining is not failed
    if (motionActivated == 0) { //when motion is not yet activated
      motionActivated = 1;  //activate motion
      counter = motionIntervalTime;
      rtc.setSeconds(5);  //set RTC time to 5 seconds to go to one minute interrupt after 1 second
    }
    intervalCurrent = motionIntervalTime; //change interval to motion interval
    motionCountDown = timeInMotionInterval + 1; //set motion countdown to configured motion interval duration
    Serial.println("Motion detected");
  }
}
void read_config_flashEEPROM() {  //function to read current settings from EEPROM is they exist
  if (!EEPROM.isValid()) {  //check for memory content
    Serial.println("Emulated EEPROM is empty"); //if memory is empty, write the in code configured settings
    EEPROM.write(0, ledActivation);
    EEPROM.write(1, uint8_t(intervalTime >> 8));
    EEPROM.write(2, uint8_t(intervalTime));
    EEPROM.write(3, WiFiActivation);
    EEPROM.write(4, GNSSActivation);
    EEPROM.write(5, motionActivation);
    EEPROM.write(6, timeInMotionInterval);
    EEPROM.write(7, uint8_t(motionIntervalTime));
    EEPROM.write(8, acc_threshold);
    EEPROM.write(9, acc_duration);
    EEPROM.commit();
    Serial.println("Written current settings to emulated EEPROM");
  }
  else {  //if memory has data, read the settings from flash / EEPROM
    Serial.println("EEPROM has data");
    ledActivation = EEPROM.read(0);
    intervalTime = ((EEPROM.read(1) << 8) | EEPROM.read(2));
    WiFiActivation = EEPROM.read(3);
    GNSSActivation = EEPROM.read(4);
    motionActivation = EEPROM.read(5);
    timeInMotionInterval = EEPROM.read(6);
    motionIntervalTime = EEPROM.read(7);
    acc_threshold = EEPROM.read(8);
    acc_duration = EEPROM.read(9);
    Serial.println("Read settings from emulated EEPROM");
  }
  Serial.print("LED activation: ");
  Serial.println(ledActivation);
  Serial.print("Interval: ");
  Serial.println(intervalTime);
  Serial.print("WiFi activation: ");
  Serial.println(WiFiActivation);
  Serial.print("GNSS activation: ");
  Serial.println(GNSSActivation);
  Serial.print("Motion activation: ");
  Serial.println(motionActivation);
  Serial.print("Motion interval:  ");
  Serial.println(motionIntervalTime);
  Serial.print("Motion interval duration: ");
  Serial.println(timeInMotionInterval);
  Serial.print("Motion threshold: ");
  Serial.println(acc_threshold);
  Serial.print("Motion duration: ");
  Serial.println(acc_duration);
}
void write_config_flashEEPROM() { //function to write current settings to EEPROM
  EEPROM.write(0, ledActivation);
  EEPROM.write(1, uint8_t(intervalTime >> 8));
  EEPROM.write(2, uint8_t(intervalTime));
  EEPROM.write(3, WiFiActivation);
  EEPROM.write(4, GNSSActivation);
  EEPROM.write(5, motionActivation);
  EEPROM.write(6, timeInMotionInterval);
  EEPROM.write(7, uint8_t(motionIntervalTime));
  EEPROM.write(8, acc_threshold);
  EEPROM.write(9, acc_duration);
  EEPROM.commit();
  Serial.println("Written new settings to emulated EEPROM");
}
void enable_alc_sync() {  //function to enable alc sync on port 202
  lr1110_modem_set_alc_sync_port(202);
  lr1110_modem_set_alc_sync_mode(LR1110_MODEM_ALC_SYNC_MODE_ENABLE);
}
void wait_on_event_pin ( uint32_t timeout_s ) { //function to wait on the event pin with timer timeout, timeout_s is in seconds
  uint32_t timer = 0; //set timer to 0
  uint32_t current = rtc.getSeconds();  //get current seconds from RTC
  while (digitalRead (eventPin) == 0) //do while no event is available
  {
    if (current != rtc.getSeconds()) {  //if current is different from RTC seconds add 1 to timer
      current = rtc.getSeconds();
      timer++;
    }
    if ( timer > timeout_s )  //if timeout value is reached, return function
    {
      Serial.println("Wait timeout");
      return;
    }
  }
  Serial.println("Wait succesfull");
  return; //return function if eventpin is high
}
