/*!
 * \file      lr1110_modem_lorawan.h
 *
 * \brief     LoRaWAN driver for LR1110 modem
 *
 * Revised BSD License
 * Copyright Semtech Corporation 2020. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH S.A. BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __LR1110_MODEM_LORAWAN_H__
#define __LR1110_MODEM_LORAWAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>
#include "lr1110_modem_common.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*!
 * \brief DM Message length
 */
#define LR1110_MODEM_DM_MESSAGE_STATUS_LENGTH ( 1 )
#define LR1110_MODEM_DM_MESSAGE_CHARGE_LENGTH ( 2 )
#define LR1110_MODEM_DM_MESSAGE_VOLTAGE_LENGTH ( 1 )
#define LR1110_MODEM_DM_MESSAGE_TEMPERATURE_LENGTH ( 1 )
#define LR1110_MODEM_DM_MESSAGE_SIGNAL_LENGTH ( 2 )
#define LR1110_MODEM_DM_MESSAGE_UPTIME_LENGTH ( 2 )
#define LR1110_MODEM_DM_MESSAGE_RXTIME_LENGTH ( 2 )
#define LR1110_MODEM_DM_MESSAGE_FIRMWARE_LENGTH ( 8 )
#define LR1110_MODEM_DM_MESSAGE_ADR_MODE_LENGTH ( 1 )
#define LR1110_MODEM_DM_MESSAGE_JOIN_EUI_LENGTH ( 8 )
#define LR1110_MODEM_DM_MESSAGE_INTERVAL_LENGTH ( 1 )
#define LR1110_MODEM_DM_MESSAGE_REGION_LENGTH ( 1 )
#define LR1110_MODEM_DM_MESSAGE_RESET_COUNT_LENGTH ( 2 )
#define LR1110_MODEM_DM_MESSAGE_DEV_EUI_LENGTH ( 8 )
#define LR1110_MODEM_DM_MESSAGE_SESSION_ID_LENGTH ( 2 )
#define LR1110_MODEM_DM_MESSAGE_CHIP_EUI ( 8 )
#define LR1110_MODEM_DM_MESSAGE_STREAM_PARAMETERS_LENGTH ( 2 )
#define LR1110_MODEM_DM_MESSAGE_APPLICATION_SPECIFIC_STATUS_LENGTH ( 8 )

/*!
 * \brief Length in bytes of a chip eui
 */
#define LR1110_MODEM_CHIP_EUI_LENGTH 0x08

/*!
 * \brief Length in bytes of a LoRaWAN network key
 */
#define LR1110_MODEM_NWK_KEY_LENGTH ( 16 )

/*!
 * \brief Length in bytes of a LoRaWAN application key
 */
#define LR1110_MODEM_APP_KEY_LENGTH ( 16 )

/*!
 * \brief Number of regions available
 */
#define LR1110_MODEM_REGIONS_NUMBER ( 2 )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * \brief Power Amplifier Selection values
 *
 * - Low-power Power Amplifier can reach up to 14dBm
 * - High-power Power Amplifier can reach up to 22 dBm
 */
typedef enum
{
    LR1110_MODEM_RADIO_PA_SEL_LP       = 0x00,  //!< Low-power Power Amplifier
    LR1110_MODEM_RADIO_PA_SEL_HP       = 0x01,  //!< High-power Power Amplifier
    LR1110_MODEM_RADIO_PA_SEL_LP_HP_LF = 0x02,  //!< Low-power & High-power Power Amplifier
} lr1110_modem_radio_pa_selection_t;

/*!
 * \brief functionality values
 */
typedef enum
{
    LR1110_MODEM_FUNCTIONALITY_TRX                   = 0x01,
    LR1110_MODEM_FUNCTIONALITY_MODEM_WIFI            = 0x02,
    LR1110_MODEM_FUNCTIONALITY_MODEM_WIFI_GPS        = 0x03,
    LR1110_MODEM_FUNCTIONALITY_MODEM_WIFI_GPS_BEIDOU = 0x04,
} lr1110_modem_functionality_t;

/*!
 * \brief LoRaWAN class type
 */
typedef enum
{
    LR1110_LORAWAN_CLASS_A = 0x00,
    LR1110_LORAWAN_CLASS_C = 0x01,
} lr1110_modem_classes_t;

/*!
 * \brief Modem status bits
 */
typedef enum
{
    LR1110_LORAWAN_BROWNOUT = 0x01,
    LR1110_LORAWAN_CRASH    = 0x02,
    LR1110_LORAWAN_MUTE     = 0x04,
    LR1110_LORAWAN_JOINED   = 0x08,
    LR1110_LORAWAN_SUSPEND  = 0x10,
    LR1110_LORAWAN_UPLOAD   = 0x20,
    LR1110_LORAWAN_JOINING  = 0x40,
    LR1110_LORAWAN_STREAM   = 0x80,
} lr1110_modem_status_t;

/*!
 * \brief LoRaWAN region type
 */
typedef enum
{
    LR1110_LORAWAN_REGION_EU868 = 0x01,
    LR1110_LORAWAN_REGION_US915 = 0x03,
} lr1110_modem_regions_t;

/*!
 * \brief Adaptative Data Rate profiles type
 */
typedef enum
{
    LR1110_MODEM_ADR_PROFILE_NETWORK_SERVER_CONTROLLED = 0x00,  //!< Network Server Controlled
    LR1110_MODEM_ADR_PROFILE_MOBILE_LONG_RANGE = 0x01,  //!< Mobile Long Range : 50% MinDr, 25% MinDr + 1, 25% MinDr + 2
    LR1110_MODEM_ADR_PROFILE_MOBILE_LOW_POWER =
        0x02,  //!< Mobile Low Power : 25% MaxDr, 25% MaxDr - 1, 25% MaxDr - 2, 25% MaxDr - 3
    LR1110_MODEM_ADR_PROFILE_CUSTOM =
        0x03,  //!< Custom List A custom ADR profile consists of a list of 16 preferred data rates.
               //!< For every transmission, a random entry in that list is selected.
} lr1110_modem_adr_profiles_t;

/*!
 * \brief DM reporting internal format
 */
typedef enum
{
    LR1110_MODEM_REPORTING_INTERVAL_IN_SECOND = 0x00,
    LR1110_MODEM_REPORTING_INTERVAL_IN_DAY    = 0x01,
    LR1110_MODEM_REPORTING_INTERVAL_IN_HOUR   = 0x02,
    LR1110_MODEM_REPORTING_INTERVAL_IN_MINUTE = 0x03,
} lr1110_modem_reporting_interval_format_t;

/*!
 * \brief DM status information code
 */
typedef enum
{
    LR1110_MODEM_DM_INFO_TYPE_STATUS                                  = 0x00,
    LR1110_MODEM_DM_INFO_TYPE_CHARGE                                  = 0x01,
    LR1110_MODEM_DM_INFO_TYPE_VOLTAGE                                 = 0x02,
    LR1110_MODEM_DM_INFO_TYPE_TEMPERATURE                             = 0x03,
    LR1110_MODEM_DM_INFO_TYPE_SIGNAL                                  = 0X04,
    LR1110_MODEM_DM_INFO_TYPE_UPTIME                                  = 0x05,
    LR1110_MODEM_DM_INFO_TYPE_RXTIME                                  = 0x06,
    LR1110_MODEM_DM_INFO_TYPE_FIRMWARE                                = 0x07,
    LR1110_MODEM_DM_INFO_TYPE_ADR_MODE                                = 0x08,
    LR1110_MODEM_DM_INFO_TYPE_JOIN_EUI                                = 0x09,
    LR1110_MODEM_DM_INFO_TYPE_INTERVAL                                = 0x0A,
    LR1110_MODEM_DM_INFO_TYPE_REGION                                  = 0x0B,
    LR1110_MODEM_DM_INFO_TYPE_CRASH_LOG                               = 0x0D,
    LR1110_MODEM_DM_INFO_TYPE_UPLOAD                                  = 0x0E,
    LR1110_MODEM_DM_INFO_TYPE_RESET_COUNT                             = 0x0F,
    LR1110_MODEM_DM_INFO_TYPE_DEV_EUI                                 = 0x10,
    LR1110_MODEM_DM_INFO_TYPE_OWNER_COUNTER                           = 0x11,
    LR1110_MODEM_DM_INFO_TYPE_SESSION_ID                              = 0x12,
    LR1110_MODEM_DM_INFO_TYPE_CHIP_EUI                                = 0x13,
    LR1110_MODEM_DM_INFO_TYPE_STREAM                                  = 0x14,
    LR1110_MODEM_DM_INFO_TYPE_STREAM_PARAMETERS                       = 0x15,
    LR1110_MODEM_DM_INFO_TYPE_APPLICATION_SPECIFIC_STATUS             = 0x16,
    LR1110_MODEM_DM_INFO_TYPE_APPLICATION_LAYER_CLOCK_SYNCHRONISATION = 0x17,
    LR1110_MODEM_DM_INFO_TYPE_GNSS_ALMANAC_STATUS                     = 0x18,
    LR1110_MODEM_DM_INFO_TYPE_GNSS_DEBUG_RESPONSE                     = 0x19,
    LR1110_MODEM_DM_INFO_TYPE_GNSS_LOC                                = 0x1A,
    LR1110_MODEM_DM_INFO_TYPE_WIFI_LOC                                = 0x1B,
} lr1110_modem_dm_info_type_t;

/*!
 * \brief LoRaWAN uplink type
 */
typedef enum
{
    LR1110_MODEM_UPLINK_UNCONFIRMED = 0x00,
    LR1110_MODEM_UPLINK_CONFIRMED   = 0x01,
} lr1110_modem_uplink_type_t;

/*!
 * \brief Radio test mode type
 */
typedef enum
{
    LR1110_MODEM_TEST_MODE_TST_START                       = 0x00,
    LR1110_MODEM_TEST_MODE_TST_NOP                         = 0x01,
    LR1110_MODEM_TEST_MODE_TST_TX_SINGLE                   = 0x02,
    LR1110_MODEM_TEST_MODE_TST_TX_CONT                     = 0x03,
    LR1110_MODEM_TEST_MODE_TST_CW                          = 0x06,
    LR1110_MODEM_TEST_MODE_TST_RX_CONT                     = 0x07,
    LR1110_MODEM_TEST_MODE_TST_RSSI                        = 0x08,
    LR1110_MODEM_TEST_MODE_TST_RADIO_RST                   = 0x09,
    LR1110_MODEM_TEST_MODE_TST_EXIT                        = 0x0B,
    LR1110_MODEM_TEST_MODE_TST_BUSY_LOOP                   = 0x0C,
    LR1110_MODEM_TEST_MODE_TST_PANIC                       = 0x0D,
    LR1110_MODEM_TEST_MODE_TST_WATCHDOG                    = 0x0E,
    LR1110_MODEM_TEST_MODE_TST_TX_SINGLE_PREAM             = 0x14,
    LR1110_MODEM_TEST_MODE_READ_RSSI                       = 0x15,
    LR1110_MODEM_TEST_MODE_TST_RSSI_2G4                    = 0x16,
    LR1110_MODEM_TEST_MODE_TST_RSSI_GNSS                   = 0x17,
    LR1110_MODEM_TEST_MODE_TST_READ_RX_PKT_COUNTER_RX_CONT = 0x18,
} lr1110_modem_test_mode_t;

/*!
 * \brief Spreading factor for test mode
 */
typedef enum
{
    LR1110_MODEM_TST_MODE_FSK  = 0x00,
    LR1110_MODEM_TST_MODE_SF7  = 0x01,
    LR1110_MODEM_TST_MODE_SF8  = 0x02,
    LR1110_MODEM_TST_MODE_SF9  = 0x03,
    LR1110_MODEM_TST_MODE_SF10 = 0x04,
    LR1110_MODEM_TST_MODE_SF11 = 0x05,
    LR1110_MODEM_TST_MODE_SF12 = 0x06,
} lr1110_modem_tst_mode_sf_t;

/*!
 * \brief Bandwidth for test mode
 */
typedef enum
{
    LR1110_MODEM_TST_MODE_125_KHZ = 0x00,
    LR1110_MODEM_TST_MODE_250_KHZ = 0x01,
    LR1110_MODEM_TST_MODE_500_KHZ = 0x02,
    LR1110_MODEM_TST_MODE_12_MHZ  = 0x0F,
    LR1110_MODEM_TST_MODE_18_MHZ  = 0x10,
    LR1110_MODEM_TST_MODE_24_MHZ  = 0x11,
} lr1110_modem_tst_mode_bw_t;

/*!
 * \brief Coding rate for test mode
 */
typedef enum
{
    LR1110_MODEM_TST_MODE_4_5 = 0x00,
    LR1110_MODEM_TST_MODE_4_6 = 0x01,
    LR1110_MODEM_TST_MODE_4_7 = 0x02,
    LR1110_MODEM_TST_MODE_4_8 = 0x03,
} lr1110_modem_tst_mode_cr_t;

/*!
 * \brief Coding rate for test mode
 */
typedef enum
{
    LR1110_MODEM_TST_MODE_CONSTELLATION_GNSS   = 0x00,
    LR1110_MODEM_TST_MODE_CONSTELLATION_BEIDOU = 0x01,
} lr1110_modem_tst_mode_constellation_t;

/*!
 * \brief Encryption mode values
 */
typedef enum
{
    LR1110_MODEM_FILE_ENCRYPTION_DISABLE = 0x00,
    LR1110_MODEM_FILE_ENCRYPTION_ENABLE  = 0x01,
} lr1110_modem_encryption_mode_t;

/*!
 * \brief LoRaWAN state values
 */
typedef enum
{
    LR1110_MODEM_LORWAN_IDLE = 0x00,
    LR1110_MODEM_LORWAN_BUSY = 0x10,
} lr1110_modem_lorawan_state_t;

/*!
 * \brief TX status values
 */
typedef enum
{
    LR1110_MODEM_UNCONFIRMED_TX = 0x01,
    LR1110_MODEM_CONFIRMED_TX   = 0x02,
    LR1110_MODEM_TX_ERROR       = 0x03,
} lr1110_modem_tx_done_event_t;

/*!
 * \brief RX flags encoding
 */
typedef enum
{
    LR1110_MODEM_DOWN_DATA_EVENT_ACK  = 0x80,  //!< confirmed UP frame was acked
    LR1110_MODEM_DOWN_DATA_EVENT_NACK = 0x40,  //!< confirmed UP frame was not acked
    LR1110_MODEM_DOWN_DATA_EVENT_DNW1 = 0x01,  //!< received in 1st DN slot
    LR1110_MODEM_DOWN_DATA_EVENT_DNW2 = 0x02,  //!< received in 2dn DN slot
} lr1110_modem_down_data_flag_t;

/*!
 * \brief Modem charge type
 */
typedef enum
{
    LR1110_MODEM_CHARGE_TYPE_MODEM        = 0x00,
    LR1110_MODEM_CHARGE_TYPE_USER_DEFINED = 0x01,
} lr1110_modem_charge_type_t;

/*!
 * \brief LoRaWAN Duty Cycle activation type
 */
typedef enum
{
    LR1110_MODEM_DUTY_CYCLE_DISABLE = 0x00,
    LR1110_MODEM_DUTY_CYCLE_ENABLE  = 0x01,
} lr1110_modem_duty_cycle_t;

/*!
 * \brief LoRaWAN Duty Cycle activation type
 */
typedef enum
{
    LR1110_MODEM_CERTIFICATION_MODE_DISABLE = 0x00,
    LR1110_MODEM_CERTIFICATION_MODE_ENABLE  = 0x01,
} lr1110_modem_certification_mode_t;

/*!
 * \brief ALC Sync service activation mode
 */
typedef enum
{
    LR1110_MODEM_ALC_SYNC_MODE_DISABLE = 0x00,
    LR1110_MODEM_ALC_SYNC_MODE_ENABLE  = 0x01,
} lr1110_modem_alc_sync_mode_t;

/*!
 * \brief ALC Sync state
 */
typedef enum
{
    LR1110_MODEM_ALC_SYNC_DESYNCHRONIZED = 0x00,
    LR1110_MODEM_ALC_SYNC_SYNCHRONIZED   = 0x01,
} lr1110_modem_alc_sync_state_t;

/*!
 * \brief Modem mute type
 */
typedef enum
{
    LR1110_MODEM_UNMUTED = 0x00,
    LR1110_MODEM_MUTED   = 0x01,
} lr1110_modem_mute_t;

/*!
 * \brief Modem suspend type
 */
typedef enum
{
    LR1110_MODEM_RESUMED = 0x00,
    LR1110_MODEM_SUSPEND = 0x01,
} lr1110_modem_suspend_t;

/*!
 * \brief modem event fields structure
 */
typedef struct
{
    lr1110_modem_lorawan_event_type_t event_type;
    uint8_t                           event_count;
    uint8_t                           buffer[LR1110_MODEM_EVENT_MAX_LENGTH_BUFFER];
    uint16_t                          buffer_len;
} lr1110_modem_event_fields_t;

/*!
 * \brief LR1110 modem version structure
 */
typedef struct
{
    uint32_t                     bootloader;
    lr1110_modem_functionality_t functionality;
    uint32_t                     firmware;
    uint16_t                     lorawan;
} lr1110_modem_version_t;

/*!
 * \brief DM info fields structure
 *
 * \note The array dm_info_field is to be populated with values from lr1110_modem_dm_info_type_t
 */
typedef struct
{
    uint8_t  dm_info_field[20];
    uint16_t dm_info_length;
} lr1110_modem_dm_info_fields_t;

/*!
 * \brief stream status structure
 */
typedef struct
{
    uint16_t pending;  //!< number of bytes pending for transmission
    uint16_t free;     //!< number of bytes still free in the buffer
} lr1110_modem_stream_status_t;

/*!
 * \brief Chip EUI type
 */
typedef uint8_t lr1110_modem_chip_eui_t[LR1110_MODEM_CHIP_EUI_LENGTH];

/*!
 * \brief LoRaWAN list of regions type
 */
typedef lr1110_modem_regions_t lr1110_modem_regions_list_t[LR1110_MODEM_REGIONS_NUMBER];

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief This command can be used to retrieve pending events from the modem. Pending events are indicated by the
 * EVENT line. The EVENT line will be de-asserted after all events have been retrieved and no further events are
 * available. When no event is available this command returns with empty response payload.
 *
 * \param [in]  Chip implementation 
 *
 * \param [out] event_fields \see lr1110_modem_event_fields_t
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_get_event(  lr1110_modem_event_fields_t* event_fields );

/*!
 * \brief This command returns the version of the bootloader and the version of the installed firmware plus the version
 * of the implemented LoRaWAN standard (BCD, e.g. 0x0103 for LoRaWAN 1.0.3).
 *
 * \param [in]  Chip implementation 
 *
 * \param [out] version Version of the LR1110 modem
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_get_version(  lr1110_modem_version_t* version );

/*!
 * \brief This command performs a reset of the LR1110 modem. All transient state (including session information) will be
 * lost and the modem needs to join the network again.
 *
 * \param [in]  Chip implementation 
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_reset(  );

/*!
 * \brief This command resets the accumulated charge counter to zero.
 *
 * \param [in]  Chip implementation 
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_reset_charge(  );

/*!
 * \brief This command returns the total charge counter of the modem in mAh. This value includes the accumulated charge
 * since the production of the modem or since the last invocation of the ResetCharge command.
 *
 * \param [in]  Chip implementation 
 *
 * \param [out] charge Charge counter in mAh
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_get_charge(  uint32_t* charge );

/*!
 * \brief This command gets the board-specific correction offset for transmission power to be used (signed integer in
 * dB).
 *
 * \param [in]  Chip implementation 
 *
 * \param [out] tx_power_offset Tx power offset correction in dBm
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_get_tx_power_offset(  int8_t* tx_power_offset );

/*!
 * \brief This command sets the board-specific correction offset for transmission power to be used. The offset depends
 * on the board design and antenna matching and is expressed in dB (signed integer).
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] tx_power_offset Tx power offset correction in dBm
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_set_tx_power_offset(  const int8_t tx_power_offset );

/*!
 * \brief This command is used to implement test functionality for regulatory conformance, certification, and functional
 * testing. With the exception of the TST_START command, test commands are only available if test mode is active. Test
 * mode can only be activated if the modem has not yet received a command that results in a radio operation. Once test
 * mode is active, all other modem commands are disabled.
 *
 * \param [in]  Chip implementation 
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_test_mode_start(  );

/*!
 * \brief No operation. This command may be used to terminate an ongoing continuous TX operation.
 *
 * \param [in]  Chip implementation 
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_test_nop(  );

/*!
 * \brief Transmit a single packet.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] frequency Frequency in Hz
 *
 * \param [in] tx_power Tx power in dBm
 *
 * \param [in] sf spreading factor \ref lr1110_modem_tst_mode_sf_t
 *
 * \param [in] bw bandwidth \ref lr1110_modem_tst_mode_bw_t
 *
 * \param [in] cr Coding Rate \ref lr1110_modem_tst_mode_cr_t
 *
 * \param [in] payload_length Payload length
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_test_tx_single(  uint32_t frequency, int8_t tx_power,
                                                          lr1110_modem_tst_mode_sf_t sf, lr1110_modem_tst_mode_bw_t bw,
                                                          lr1110_modem_tst_mode_cr_t cr, uint8_t payload_length );

/*!
 * \brief Continuously transmit packets as fast as possible.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] frequency Frequency in Hz
 *
 * \param [in] tx_power Tx power in dBm
 *
 * \param [in] sf spreading factor \ref lr1110_modem_tst_mode_sf_t
 *
 * \param [in] bw bandwidth \ref lr1110_modem_tst_mode_bw_t
 *
 * \param [in] cr Coding Rate \ref lr1110_modem_tst_mode_cr_t
 *
 * \param [in] payload_length Payload length
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_test_tx_cont(  uint32_t frequency, int8_t tx_power,
                                                        lr1110_modem_tst_mode_sf_t sf, lr1110_modem_tst_mode_bw_t bw,
                                                        lr1110_modem_tst_mode_cr_t cr, uint8_t payload_length );

/*!
 * \brief Transmit a continuous wave.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] frequency Frequency in Hz
 *
 * \param [in] tx_power Tx power in dBm
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_test_tx_cw(  uint32_t frequency, int8_t tx_power );

/*!
 * \brief Continuously receive packets.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] frequency Frequency in Hz
 *
 * \param [in] sf spreading factor \ref lr1110_modem_tst_mode_sf_t
 *
 * \param [in] bw bandwidth \ref lr1110_modem_tst_mode_bw_t
 *
 * \param [in] cr Coding Rate \ref lr1110_modem_tst_mode_cr_t
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_test_rx_cont(  uint32_t frequency,
                                                        lr1110_modem_tst_mode_sf_t sf, lr1110_modem_tst_mode_bw_t bw,
                                                        lr1110_modem_tst_mode_cr_t cr );

/*!
 * \brief Read the packet counter received during continuously receive packets test.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] rx_packet_counter The counter of packet received during RX continuous packet test
 *
 * \returns Operation status
 *
 * \see lr1110_modem_test_rx_cont
 */
lr1110_modem_response_code_t lr1110_modem_test_read_packet_counter_rx_cont( 
                                                                            uint32_t*   rx_packet_counter );

/*!
 * \brief Continuously receive packets.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] frequency Frequency in Hz
 *
 * \param [in] time_ms time in millisecond of the radio acquisition
 *
 * \param [in] bw bandwidth \ref lr1110_modem_tst_mode_bw_t
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_test_rssi(  uint32_t frequency, uint16_t time_ms,
                                                     lr1110_modem_tst_mode_bw_t bw );

/*!
 * \brief Reset the LR1110 radio.
 *
 * \param [in]  Chip implementation 
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_test_radio_rst(  );

/*!
 * \brief Exit test mode and reset LR1110 modem.
 *
 * \param [in]  Chip implementation 
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_test_exit(  );

/*!
 * \brief Causes to modem to enter an endless loop with interrupts enabled. Eventually, the software watchdog will save
 * a crashlog and reset the device. Note that this command will render the modem completely unresponsive until it is
 * reset.
 *
 * \param [in]  Chip implementation 
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_test_busy_loop(  );

/*!
 * \brief Causes an unrecoverable fault condition (panic). The modem will immediately save a crashlog and halt, the
 * diagnostics LED (if available) will flash for some time, and then the modem will reset. Note that this command will
 * render the modem completely unresponsive until it is reset.
 *
 * \param [in]  Chip implementation 
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_test_panic(  );

/*!
 * \brief Causes to modem to enter an endless loop with interrupts disabled. Eventually, the hardware watchdog will
 * reset the device. No crashlog will be written. Note that this command will render the modem completely unresponsive
 * until it is reset.
 *
 * \param [in]  Chip implementation 
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_test_watchdog(  );

/*!
 * \brief Transmit a single packet with the number of preamble configurable.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] frequency Frequency in Hz
 *
 * \param [in] tx_power Tx power in dBm
 *
 * \param [in] sf spreading factor \ref lr1110_modem_tst_mode_sf_t
 *
 * \param [in] bw bandwidth \ref lr1110_modem_tst_mode_bw_t
 *
 * \param [in] cr Coding Rate \ref lr1110_modem_tst_mode_cr_t
 *
 * \param [in] payload_length Payload length
 *
 * \param [in] preamble_length Preamble Length
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_test_tx_single_preamble(  uint32_t frequency,
                                                                   int8_t tx_power, lr1110_modem_tst_mode_sf_t sf,
                                                                   lr1110_modem_tst_mode_bw_t bw,
                                                                   lr1110_modem_tst_mode_cr_t cr,
                                                                   uint8_t payload_length, uint16_t preamble_length );

/*!
 * \brief Read RSSI after a Sub Gig / 2.4 Ghz / GNSS test rssi command.
 *
 * \param [in]  Chip implementation 
 *
 * \param [out] rssi RSSI in dBm
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_test_read_rssi(  int8_t* rssi );

/*!
 * \brief Continuously receive packets on 2.4GHz radio path.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] channel Wi-Fi Channel from 1 to 14
 *
 * \param [in] time_ms time in millisecond of the radio acquisition
 *
 * \param [in] bw bandwidth \ref lr1110_modem_tst_mode_bw_t \note Allowed 12MHz and 24MH
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_test_rssi_2g4(  uint8_t channel, uint16_t time_ms,
                                                         lr1110_modem_tst_mode_bw_t bw );

/*!
 * \brief Continuously receive packets on GNSS radio path.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] constellation_modulation constellation used \ref lr1110_modem_tst_mode_constellation_t
 *
 * \param [in] time_ms time in millisecond of the radio acquisition
 *
 * \param [in] bw bandwidth \ref lr1110_modem_tst_mode_bw_t \note Allowed 12MHz
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_test_rssi_gnss(
     lr1110_modem_tst_mode_constellation_t constellation_modulation, uint16_t time_ms,
    lr1110_modem_tst_mode_bw_t bw );

/*!
 * \brief Query the current GPS time. The application layer clock synchronization protocol is used to link the device
 * clock to GPS time. The returned time specifies the seconds since GPS epoch (00:00:00, Sunday 6th of January 1980). If
 * the device is not yet synchronized to GPS time then the returned value is zero. This may happen if the server has not
 * yet answered time sync requests. The accuracy of the synchronization is in the range of seconds and depends on
 * latencies in the network infrastructure.
 *
 * \param [in]  Chip implementation 
 *
 *
 * \param [out] time Time in seconds
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_get_gps_time(  uint32_t* time );

/*!
 * \brief This command returns the modem status which may indicate one or more notification conditions.
 *
 * \param [in]  Chip implementation 
 *
 * \param [out] status LR1110 mode status but mask \ref lr1110_modem_status_t
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_get_status(  lr1110_modem_status_t* status );

/*!
 * \brief This command sets an application alarm timer (in seconds). When the timer has expired an Alarm event is
 * generated. If this command is applied again before the timer has expired, the timer will be started again with the
 * new period. A value of 0 will cancel an possibly pending previous alarm timer.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] seconds Seconds
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_set_alarm_timer(  uint32_t seconds );

/*!
 * \brief This command returns the device registration PIN
 *
 * \param [in]  Chip implementation 
 *
 * \param [out] pin Device registration PIN on 4 bytes
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_get_pin(  uint32_t* pin );

/*!
 * \brief This command returns the ChipEUI. The ChipEUI is also the default DeviceEUI. It is programmed during
 * manufacturing and is immutable.
 *
 * \param [in]  Chip implementation 
 *
 * \param [out] chip_eui Chip EUI buffer on 8 bytes
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_get_chip_eui(  lr1110_modem_chip_eui_t chip_eui );

/*!
 * \brief This command returns the join EUI.
 *
 * \note Join EUI is also known as Application Identifier (AppEUI) in LoRaWan v1.0.3.
 *
 * \param [in]  Chip implementation 
 *
 * \param [out] join_eui Join EUI buffer on 8 bytes
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_get_join_eui(  uint8_t* join_eui );

/*!
 * \brief This command sets the Join EUI.
 *
 * \note Join EUI is also known as Application Identifier (AppEUI) in LoRaWan v1.0.3.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] join_eui Join EUI buffer on 8 bytes
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_set_join_eui(  const uint8_t* join_eui );

/*!
 * \brief This command returns the DeviceEUI.
 *
 * \param [in]  Chip implementation 
 *
 * \param [out] dev_eui Device EUI buffer on 8 bytes
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_get_dev_eui(  uint8_t* dev_eui );

/*!
 * \brief This command sets the DeviceEUI.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] dev_eui Device EUI buffer on 8 bytes
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_set_dev_eui(  const uint8_t* dev_eui );

/*!
 * \brief This command sets the LoRaWAN 1.0.3 application key. Note that a factory reset will erase this information.
 * The device is required to rejoin the network.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] app_key Application Key buffer on 16 bytes
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_set_app_key(  const uint8_t* app_key );

/*!
 * \brief This command gets the LoRaWAN device class.
 *
 * \param [in]  Chip implementation 
 *
 * \param [out] modem_class LoRaWAN device class
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_get_class(  lr1110_modem_classes_t* modem_class );

/*!
 * \brief This command sets the LoRaWAN device class. Currently only class A and class C are supported. If the command
 * is successful, a change from class A to class C is effective after a completed TX transaction. The network server
 * should also be informed about the class change, typically on a separate channel for LoRaWAN 1.0.3. For a change from
 * class C to class A, the RX remains enabled until the next TX transaction.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] modem_class LoRaWAN device class \ref lr1110_modem_classes_t
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_set_class(  const lr1110_modem_classes_t modem_class );

/*!
 * \brief This command returns the regulatory region.
 *
 * \param [in]  Chip implementation 
 *
 * \param [out] region LoRaWAN regulatory region \ref lr1110_modem_regions_t
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_get_region(  lr1110_modem_regions_t* region );

/*!
 * \brief This command sets the regulatory region. Additionally this command resets the ADR profile to Network Server
 * Controlled. If different ADR profile is desired, the profile needs to be set again.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] region LoRaWAN regulatory region \ref lr1110_modem_regions_t
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_set_region(  const lr1110_modem_regions_t region );

/*!
 * \brief This command returns the regulatory regions supported by the LR1110 modem
 *
 * \param [in]  Chip implementation 
 *
 * \param [out] region_codes Regions supported by the LR1110 modem
 *
 * \param [out] region_codes_size size of the regions list
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_list_regions(  lr1110_modem_regions_list_t region_codes,
                                                        uint8_t* region_codes_size );

/*!
 * \brief This command returns the ADR profile type.
 *
 * \param [in]  Chip implementation 
 *
 * \param [out] adr_profile ADR profile type \ref lr1110_modem_adr_profiles_t
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_get_adr_profile(                 
                                                           lr1110_modem_adr_profiles_t* adr_profile );

/*!
 * \brief This command sets the ADR profile and parameters.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] adr_profile ADR profile type \ref lr1110_modem_adr_profiles_t
 *
 * \param [in] adr_custom_list custom ADR profile consisting of a list of 16 preferred data rates
 *
 * \note each call to the function reinitialize the data rate distribution.
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_set_adr_profile(                       
                                                           const lr1110_modem_adr_profiles_t adr_profile,
                                                           const uint8_t*                    adr_custom_list );

/*!
 * \brief This command gets the device management port.
 *
 * \param [in]  Chip implementation 
 *
 * \param [out] port Device management port
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_get_dm_port(  uint8_t* port );

/*!
 * \brief This command sets the device management port. Port 0 and ports from 224 and 255 must not be used since they
 * are reserved for future standardized application extensions.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] port Device management port
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_set_dm_port(  const uint8_t port );

/*!
 * \brief This command returns the device management reporting interval. The interval is specified in seconds, minutes,
 * hours or days.
 *
 * \param [in]  Chip implementation 
 *
 * \param [out] interval interval specified in seconds, minutes, hours or days
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_get_dm_info_interval(  uint8_t* interval );

/*!
 * \brief This command sets the device management reporting interval. The interval is specified in seconds, minutes,
 * hours or days.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] format reporting interval format \ref lr1110_modem_reporting_interval_format_t
 *
 * \param [in] interval interval specified in seconds, minutes, hours or days
 *
 * \note Any value of 0 (seconds, minutes, hours, or days) disables device management reporting.
 *       The periodic status reporting interval field is encoded in one byte where the two top-most bits specify the
 * unit (sec/min/hour/day), and the lower six bits the value 0-63. A value of zero disables the periodic status
 * reporting.
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_set_dm_info_interval(                                    
                                                                const lr1110_modem_reporting_interval_format_t format,
                                                                const uint8_t interval );

/*!
 * \brief This command lists the info fields to be included in the periodic DM status messages.
 *
 * \param [in]  Chip implementation 
 *
 * \param [out] dm_info_fields list of tag bytes \ref lr1110_modem_dm_info_fields_t
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_get_dm_info_field(                    
                                                             lr1110_modem_dm_info_fields_t* dm_info_fields );

/*!
 * \brief This command sets the default info fields to be included in the periodic DM status messages. The set is
 * specified as list of field codes as defined in Uplink Message Format. Duplicate and invalid fields will be rejected
 * An empty set is valid and will effectively disable the DM status message.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] dm_info_fields list of tag bytes \ref lr1110_modem_dm_info_fields_t
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_set_dm_info_field(                         
                                                             const lr1110_modem_dm_info_fields_t* dm_info_fields );

/*!
 * \brief This command sends the specified set of information fields in one or more DM status messages immediately. The
 * set is specified as list of field codes as defined in Uplink Message Format. Duplicate and invalid fields will be
 * rejected (see note in Periodic Status Reporting).
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] dm_info_fields list of tag bytes \ref lr1110_modem_dm_info_fields_t
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_send_dm_status(                   
                                                          lr1110_modem_dm_info_fields_t dm_info_fields );

/*!
 * \brief This commands sets application-specific status information to be reported to the DM service. This information
 * is an application-defined, arbitrary 8-byte data blob. Once set, it is included in the appstatus info field sent as
 * part of the periodic status reports to the DM service. On the cloud side, this information can then be retrieved from
 * the service.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] app_status Application-specific status information
 *
 * \note Note that this command does not trigger an immediate status report. If the application desires
 * to send the status immediately, it can issue the SendDmStatus command with the appstatus tag.
 * The application status is not stored persistently, i.e. after reset no application status will be reported.
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_set_app_status(  uint8_t* app_status );

/*!
 * \brief This command starts joining the network. During the join procedure no further transmissions can occur. When
 * the network has been successfully joined, a Joined event is generated. If the device is already joined to a network,
 * or is in the process of joining, this command has no effect.
 *
 * \param [in]  Chip implementation 
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_join(  );

/*!
 * \brief This command leaves the network if already joined, or cancels an ongoing join process. After leaving the
 * network, no further transmissions can occur.
 *
 * \param [in]  Chip implementation 
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_leave_network(  );

/*!
 * \brief This command temporarily suspends or resumes the modem’s radio operations. It can be used to prevent extra
 * power consumption by the modem in case the application MCU temporarily needs more power itself and wants to prevent
 * exceeding limits.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] suspend Operations are suspended with parameter value 0x01 and resumed with parameter value 0x00 \ref
 * lr1110_modem_suspend_t
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_suspend(  const lr1110_modem_suspend_t suspend );

/*!
 * \brief This command returns the maximum application payload size possible according to the LoRaWAN regional
 * parameters for the next transmission using the current data rate, while assuming no FOpts are present and that a
 * device is not behind a repeater.
 *
 * \param [in]  Chip implementation 
 *
 * \param [out] tx_max_payload Maximum application payload size possible
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_get_next_tx_max_payload(  uint8_t* tx_max_payload );

/*!
 * \brief This command requests sending the given data on the specified port as an unconfirmed (0x00) or confirmed
 * (0x01) frame. The request will be queued and the frame will be sent as soon as the current bandwidth usage of the
 * regulatory region permits. A TxDone event is generated when the frame either has been sent, or if it couldn’t be sent
 * because the specified data exceeded the maximum possible payload size.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] port LoRaWAN port
 *
 * \param [in] uplink_type Uplink type unconfirmed (0x00) or confirmed (0x01) \ref lr1110_modem_uplink_type_t
 *
 * \param [in] data Data buffer
 *
 * \param [in] length Data length
 *
 * \note The application shall not use port 0 or the LoRaWAN test port 224 as well as the ports from 225 to 255 since
 * they are reserved for future standardized application extensions.
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_request_tx(  const uint8_t port,
                                                      const lr1110_modem_uplink_type_t uplink_type, const uint8_t* data,
                                                      const uint8_t length );

/*!
 * \brief This command sends the given data on the specified port as an unconfirmed (0x00) or confirmed (0x01) frame
 * immediately. It has higher priority than all other services and does not take duty cycle or payload size restrictions
 * into account
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] port LoRaWAN port
 *
 * \param [in] uplink_type Uplink type unconfirmed (0x00) or confirmed (0x01) \ref lr1110_modem_uplink_type_t
 *
 * \param [in] data Data buffer
 *
 * \param [in] length Data length
 *
 * \note The application shall not use port 0 or the LoRaWAN test port 224 as well as the ports from 225 to 255 since
 * they are reserved for future standardized application extensions.
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_emergency_tx(  const uint8_t port,
                                                        const lr1110_modem_uplink_type_t uplink_type,
                                                        const uint8_t* data, const uint8_t length );

/*!
 * \brief This command prepares a fragmented file upload. It specifies the port for the subsequent upload, optional
 * encryption mode, file size, and average frame transmission interval (in seconds).
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] port LoRaWAN port
 *
 * \param [in] encryption_mode Encryption mode \ref lr1110_modem_encryption_mode_t
 *
 * \param [in] size Upload size
 *
 * \param [in] interval Transmission interval in seconds
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_upload_init(  const uint8_t port,
                                                       const lr1110_modem_encryption_mode_t encryption_mode,
                                                       const uint16_t size, const uint16_t interval );

/*!
 * \brief This command can be used to repeatedly set file data to be uploaded. The file data needs to be split into
 * parts of maximum 255 bytes each and the submitted parts will be appended to an internal buffer. In total exactly as
 * many bytes as specified by the UploadInit command have to be provided. The buffer allocated for file uploads is 8K
 * bytes.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] data File data to be uploaded
 *
 * \param [in] length Data length
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_upload_data(  const uint8_t* data, const uint8_t length );

/*!
 * \brief After all data bytes indicated to UploadInit have been provided using UploadData this command can be issued to
 * actually start the transmission stream.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] crc CRC parameter which must match the CRC of the supplied data
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_upload_start(  const uint32_t crc );

/*!
 * \brief This command initializes redundant data streaming on a specific port. The StreamInit command can only be
 * issued before the stream has been started using the SendStreamData command
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] port Streaming port
 *
 * \param [in] encryption_mode Encryption mode \ref lr1110_modem_encryption_mode_t
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_stream_init(  const uint8_t port,
                                                       const lr1110_modem_encryption_mode_t encryption_mode );

/*!
 * \brief This command adds a new data record to the buffer of the data streaming encoder for the given port. Whenever
 * the buffer contains data records, the modem autonomously retrieves data from the buffer, optionally encrypts it, adds
 * redundancy, and sends uplinks containing the redundant stream
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] port Streaming port
 *
 * \param [in] record File data to be streamed
 *
 * \param [in] length Data length. The maximum length is 254 bytes.
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_send_stream_data(  const uint8_t port,
                                                            const uint8_t* record, const uint8_t length );

/*!
 * \brief This command queries the status of the data streaming buffer on the specified port. It returns two unsigned 16
 * bit integer values indicating the number of bytes pending for transmission and the number of bytes still free in the
 * buffer.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] port Streaming port
 *
 * \param [in] stream_status Streaming status \ref lr1110_modem_stream_status_t
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_stream_status(  const uint8_t port,
                                                         lr1110_modem_stream_status_t* stream_status );

/*!
 * \brief This commands sends the GPS time to LR1110 Modem, The time format is 4 bytes, big endian.
 *  It encodes the number of seconds from the 6-Jan 1980 00:00:00.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] gps_time GPS Time in seconds
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_set_gps_time(  const uint32_t gps_time );

/*!
 * \brief Use the previously set of JoinEUI/DevEUI to derive the app keys used in the Semtech join server
 *
 * \param [in]  Chip implementation 
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_derive_keys(  );

/*!
 * \brief Configure rf output configuration
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] output Rf output \ref lr1110_modem_radio_pa_selection_t
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_set_rf_output(                            
                                                         const lr1110_modem_radio_pa_selection_t output );

/*!
 * \brief Set the port for application layer clock synchronization
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] port Port application
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_set_alc_sync_port(  const uint8_t port );

/*!
 * \brief Get the port for application layer clock synchronization
 *
 * \param [in]  Chip implementation 
 *
 * \param [out] port Port application
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_get_alc_sync_port(  uint8_t* port );

/*!
 * \brief Set the mode for application layer clock synchronization service
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] mode application layer clock mode
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_set_alc_sync_mode( 
                                                             const lr1110_modem_alc_sync_mode_t mode );

/*!
 * \brief Get the mode for application layer clock synchronization service
 *
 * \param [in]  Chip implementation 
 *
 * \param [out] mode application layer clock mode
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_get_alc_sync_mode(  lr1110_modem_alc_sync_mode_t* mode );

/*!
 * \brief Set the number of uplink without ack from network before Modem changes it's ADR or resets \note It is
 * recommended to have the first counter smaller than the second one. The value 0 deactivate the recovery function.
 * Default values are 255 for nb_uplink_mobile_static and 2400 for nb_uplink_reset.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] nb_uplink_mobile_static number of uplink without ack from network before by then the modem adr profile
 * will switch from mobile to static
 *
 * \param [in] nb_uplink_reset number of uplink without ack from network before Modem resets
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_set_connection_timeout( 
                                                                  const uint16_t nb_uplink_mobile_static,
                                                                  const uint16_t nb_uplink_reset );

/*!
 * \brief Get the number of uplink without ack from network before Modem changes it's ADR or resets
 *
 * \param [in]  Chip implementation 
 *
 * \param [out] nb_uplink_mobile_static number of uplink without ack from network before by then the modem adr profile
 * will switch from mobile to static
 *
 * \param [out] nb_uplink_reset number of uplink without ack from network before Modem resets
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_get_connection_timeout( 
                                                                  uint16_t*   nb_uplink_mobile_static,
                                                                  uint16_t*   nb_uplink_reset );
/*!
 * \brief Returns the LoRaWAN state Idle or Not Idle.
 *
 * \param [in]  Chip implementation 
 *
 * \param [out] lorawan_state LoRaWAN state
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_get_lorawan_state(                  
                                                             lr1110_modem_lorawan_state_t* lorawan_state );

/*!
 * \brief Write a charge value to user defined charger counter.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] charge user defined charger counter
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_write_user_defined_charge_counter(    
                                                                             const uint16_t charge );

/*!
 * \brief Read a charge value to user defined charger counter.
 *
 * \param [in]  Chip implementation 
 *
 * \param [out] charge user defined charger counter
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_read_user_defined_charge_counter(  uint16_t* charge );

/*!
 * \brief Select which charge counter to send.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] charge_type charge type \ref lr1110_modem_charge_type_t
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_select_charge_uplink(                    
                                                                const lr1110_modem_charge_type_t charge_type );

/*!
 * \brief Get Duty cycle status info.
 *
 * \param [in]  Chip implementation 
 *
 * \param [out] duty_cycle time in milli sec befote the possibility to transmit again
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_get_duty_cycle_status(  uint32_t* duty_cycle );

/*!
 * \brief Activate/deactivate Duty cycle.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] duty_cycle \ref lr1110_modem_duty_cycle_t
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_activate_duty_cycle(                    
                                                               const lr1110_modem_duty_cycle_t duty_cycle );

/*!
 * \brief Activate/deactivate the certification mode.
 *
 * \param [in]  Chip implementation 
 *
 * \param [in] enable \ref lr1110_modem_certification_mode_t
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_set_certification_mode(                           
                                                                  const lr1110_modem_certification_mode_t enable );

/*!
 * \brief Get the certification mode.
 *
 * \param [in]  Chip implementation 
 *
 * \param [out] enable \ref lr1110_modem_certification_mode_t
 *
 * \returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_get_certification_mode( 
                                                                  lr1110_modem_certification_mode_t* enable );

#ifdef __cplusplus
}
#endif

#endif  // __LR1110_MODEM_LORAWAN_H__

/* --- EOF ------------------------------------------------------------------ */
