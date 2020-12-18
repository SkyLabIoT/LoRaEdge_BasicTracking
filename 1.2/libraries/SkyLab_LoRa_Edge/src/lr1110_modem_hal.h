/*!
 * \file      lr1110_modem_hal.h
 *
 * \brief     Hardware Abstraction Layer (HAL) interface for LR1110 modem
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

#ifndef __LR1110_MODEM_HAL_H__
#define __LR1110_MODEM_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>
#include "lr1110_modem_common.h"
#include "lr1110_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

typedef enum lr1110_modem_hal_status_e
{
    LR1110_MODEM_HAL_STATUS_OK           = 0x00,
    LR1110_MODEM_HAL_STATUS_ERROR        = 0x01,
    LR1110_MODEM_HAL_STATUS_BAD_FRAME    = 0xF,
    LR1110_MODEM_HAL_STATUS_BUSY_TIMEOUT = 0xFF,
} lr1110_modem_hal_status_t;

/*
 * ============================================================================
 * API definitions to be implemented by the user
 * ============================================================================
 */

/*!
 * \brief Return the computed CRC
 *
 * \param [in] crc_initial_value initial value of the CRC
 *
 * \param [in] buffer Buffer used to compute the CRC
 *
 * \param [out] crc CRC computed
 */

inline static uint8_t lr1110_modem_compute_crc( const uint8_t crc_initial_value, const uint8_t* buffer,
                                                uint16_t length )
{
    uint8_t crc = crc_initial_value;
    uint8_t extract;
    uint8_t sum;
    for( int i = 0; i < length; i++ )
    {
        extract = *buffer;
        for( uint8_t i = 8; i; i-- )
        {
            sum = ( crc ^ extract ) & 0x01;
            crc >>= 1;
            if( sum )
            {
                crc ^= 0x65;
            }
            extract >>= 1;
        }
        buffer++;
    }
    return crc;
}

/*!
 * Radio data transfer - write
 *
 * \remark Must be implemented by the upper layer
 *
 * \param [in]           Radio implementation parameters
 * \param [in] command          Pointer to the buffer to be transmitted
 * \param [in] command_length   Buffer size to be transmitted
 * \param [in] data             Pointer to the buffer to be transmitted
 * \param [in] data_length      Buffer size to be transmitted
 *
 * \retval status     Operation status
 */
lr1110_modem_hal_status_t lr1110_modem_hal_write(  const uint8_t* command,
                                                  const uint16_t command_length, const uint8_t* data,
                                                  const uint16_t data_length );

/*!
 * Radio data transfer - read
 *
 * \remark Must be implemented by the upper layer
 *
 * \param [in]           Radio implementation parameters
 * \param [in] command          Pointer to the buffer to be transmitted
 * \param [in] command_length   Buffer size to be transmitted
 * \param [out] data             Pointer to the buffer to be received
 * \param [in] data_length      Buffer size to be received
 *
 * \retval status     Operation status
 */
lr1110_modem_hal_status_t lr1110_modem_hal_read(  const uint8_t* command,
                                                 const uint16_t command_length, uint8_t* data,
                                                 const uint16_t data_length );

/*!
 * \brief  Radio data transfer - write & read in single operation
 *
 * \remark Must be implemented by the upper layer
 * \remark Only required by lr1110_system_get_status command
 *
 * \param [in]           Radio implementation parameters
 * \param [in] command          Pointer to the buffer to be transmitted
 * \param [out] data             Pointer to the buffer to be received
 * \param [in] data_length      Buffer size to be received
 *
 * \retval status     Operation status
 */
lr1110_modem_hal_status_t lr1110_modem_hal_write_read(  const uint8_t* command, uint8_t* data,
                                                       const uint16_t data_length );

/*!
 * Reset the radio
 *
 * \remark Must be implemented by the upper layer
 *
 * \param [in]  Radio implementation parameters
 *
 * \retval status     Operation status
 */
lr1110_modem_hal_status_t lr1110_modem_hal_reset(  );

/*!
 * Switch the radio in DFU mode
 *
 * \remark Must be implemented by the upper layer
 *
 * \param [in]  Radio implementation parameters
 */
void lr1110_modem_hal_enter_dfu(  );

/*!
 * Wake the radio up.
 *
 * \remark Must be implemented by the upper layer
 *
 * \param [in]  Radio implementation parameters

 * \retval status    Operation status
 */
lr1110_modem_hal_status_t lr1110_modem_hal_wakeup(  );

#ifdef __cplusplus
}
#endif

#endif  // __LR1110_MODEM_HAL_H__
