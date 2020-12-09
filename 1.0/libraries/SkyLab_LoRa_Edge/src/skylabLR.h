/*
Libary for the Skylab LR1110 compatible board.
Jordy Vermeulen, 12 november 2020
Skylab
*/

#ifndef skylabLR_h
#define skylabLR_h

#include "Arduino.h"
#include <SPI.h>
#include "lr1110_modem_system.h"
#include "lr1110_modem_common.h"
#include "lr1110_modem_lorawan.h"
#include "lr1110_modem_driver_version.h"
#include "lr1110_hal.h"
#include "lr1110_bootloader.h"
#include "lr1110_modem_gnss.h"
#include "lr1110_modem_hal.h"
#include "lr1110_modem_wifi.h"
#include "lr1110_bootloader_types.h"
#include "lr1110_types.h"


/*define pins*/
#define NSS NSS//7 NSS
#define snifLED LEDR //4 LEDR
#define snifLED1 LEDB //4 LEDB
#define snifLED2 LEDG //4 LEDG
#define busyPin BUSY//3 BUSY
#define LNA LNA//LNA//A3  LNA
#define RESET NRESET//A0  NRESET
#define eventPin EVENT//A2 EVENT
#define batPin AIN5//A5 AIN5

#define WIFI_MAX_BASIC_RESULTS_PER_SCAN 255
#define GNSS_BUFFER_MAX_SIZE 255
#define BOARD_TCXO_WAKEUP_TIME 5  //time to wake up the tcxo oscilator

#define ASSISTED_MODE 1
#define AUTONOMOUS_MODE 2

#define LORAWAN_DEFAULT_DATARATE LR1110_MODEM_ADR_PROFILE_NETWORK_SERVER_CONTROLLED
#define LORAWAN_APP_DATA_MAX_SIZE 100  //Max bytes per uplink payload
#define LORAWAN_CONFIRMED_MSG_ON false  //ack messages for uplinks
#define LORAWAN_DUTYCYCLE_ON LR1110_MODEM_DUTY_CYCLE_ENABLE //enable LoRaWAN dutycycle


/*type definitions*/
typedef uint8_t wifi_scan_result_t;
typedef enum {
  WIFI_INIT,
  WIFI_SCAN,
  WIFI_WAIT_FOR_SCAN,
  WIFI_GET_RESULTS,
} wifi_state_t;
typedef struct {
  bool                                 enabled;
  lr1110_modem_wifi_channel_mask_t     channels;
  lr1110_modem_wifi_signal_type_scan_t types;
  lr1110_modem_wifi_mode_t             scan_mode;
  uint8_t                              nbr_retrials;
  uint8_t                              max_results;
  uint32_t                             timeout;
  lr1110_modem_wifi_result_format_t    result_format;
} wifi_settings_t;
typedef struct {
  lr1110_modem_wifi_mac_address_t        mac_address;
  lr1110_modem_wifi_channel_t            channel;
  lr1110_modem_wifi_signal_type_result_t type;
  int8_t                                 rssi;
  int16_t                                phi_offset;
  uint64_t                               timestamp_us;
  uint16_t                               beacon_period_tu;
  uint8_t                                country_code[LR1110_MODEM_WIFI_STR_COUNTRY_CODE_SIZE];
} wifi_scan_single_result_t;
typedef struct {
  uint8_t                                nbrResults;
  wifi_scan_single_result_t              results[WIFI_MAX_BASIC_RESULTS_PER_SCAN];
  lr1110_modem_wifi_cumulative_timings_t timings;
  uint32_t                               global_consumption_uas;
  uint8_t                                raw_buffer[288];
  uint16_t                               raw_buffer_size;
  bool                                   error;
} wifi_scan_all_result_t;
typedef struct {
  lr1110_modem_system_reg_mode_t reg_mode;
  wifi_scan_all_result_t         results;
  wifi_settings_t                settings;
  wifi_state_t                   state;
} wifi_t;
typedef struct {
  void ( *reset )( uint16_t reset_count );
  void ( *alarm )( void );
  void ( *joined )( void );
  void ( *join_fail )( void );
  void ( *tx_done )( lr1110_modem_tx_done_event_t status );
  void ( *down_data )( int8_t rssi, int8_t snr, lr1110_modem_down_data_flag_t flags, uint8_t port, const uint8_t* payload, uint8_t size );
  void ( *upload_done )( uint8_t session_id, uint8_t session_counter );
  void ( *set_conf )( uint8_t info_tag );
  void ( *mute )( lr1110_modem_mute_t mute );
  void ( *stream_done )( void );
  void ( *gnss_scan_done )( uint8_t* nav_message, uint16_t size );
  void ( *wifi_scan_done )( uint8_t* scan, uint16_t size );
  void ( *time_updated_alc_sync )( lr1110_modem_alc_sync_state_t alc_sync_state );
  void ( *adr_mobile_to_static )( void );
  void ( *new_link_adr )( void );
  void ( *no_event )( void );
} lr1110_modem_event_t;
typedef enum {
  GNSS_START_SCAN,
  GNSS_GET_RESULTS,
  GNSS_TERMINATED,
  GNSS_LOW_POWER,
} gnss_state_t;
typedef enum {
  GNSS_PATCH_ANTENNA = 1,
  GNSS_PCB_ANTENNA,
} antenna_t;
typedef struct {
  bool                                           enabled;
  uint8_t                                        scan_type;
  uint8_t                                        inter_capture_delay_second;
  lr1110_modem_gnss_search_mode_t                search_mode;
  uint8_t                                        input_paramaters;
  uint8_t                                        constellation_to_use;
  lr1110_modem_gnss_solver_assistance_position_t assistance_position;
  uint8_t                                        nb_sat;
} gnss_settings_t;
typedef struct {
  bool                                   double_scan_first_scan_done;
  uint16_t                               result_size;
  antenna_t                              antenna;
  uint8_t                                result_buffer[GNSS_BUFFER_MAX_SIZE];
  uint8_t                                nb_detected_satellites;
  lr1110_modem_gnss_detected_satellite_t detected_satellites[32];
} gnss_scan_single_result_t;
typedef struct {
  gnss_scan_single_result_t capture_result;
  gnss_settings_t           settings;
  gnss_state_t              state;
} gnss_t;


uint8_t read_battery();
void LR1110_join();
void LR1110_rejoin();
void LR1110_init();


#endif
