#include <SPI.h>
#include <stdlib.h>

#include "lr1110_hal.h"
#include "lr1110_modem_system.h"
#include "skylabLR.h"

static lr1110_hal_status_t lr1110_hal_wait_on_busy();

lr1110_hal_status_t lr1110_hal_write(  const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length )
{
    if( lr1110_hal_wakeup(  ) == LR1110_HAL_STATUS_OK )
    {
        digitalWrite(NSS, 0);
        for( uint16_t i = 0; i < command_length; i++ )
        {
            SPI.transfer(command[i]);
        }
        for( uint16_t i = 0; i < data_length; i++ )
        {
            SPI.transfer(data[i]);
        }
        digitalWrite(NSS, 1);

        // 0x011B - LR1110_SYSTEM_SET_SLEEP_OC
        //if( ( ( command[0] << 8 ) | command[1] ) != 0x011B )
        //{
        return lr1110_hal_wait_on_busy();
        //}
        //else
        //{
            return LR1110_HAL_STATUS_OK;
        //}
    }
    return LR1110_HAL_STATUS_ERROR;
}

lr1110_hal_status_t lr1110_hal_read( const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length )
{
    if( lr1110_hal_wakeup( ) == LR1110_HAL_STATUS_OK )
    {
        digitalWrite(NSS, 0);

        for( uint16_t i = 0; i < command_length; i++ )
        {
            SPI.transfer(command[i]);
        }

        digitalWrite(NSS, 1);

        lr1110_hal_wait_on_busy();

        // Send dummy byte
        digitalWrite(NSS, 0);
        SPI.transfer(0x00);

        for( uint16_t i = 0; i < data_length; i++ )
        {
            data[i] = SPI.transfer(0x00);
        }

        digitalWrite(NSS, 1);

        return lr1110_hal_wait_on_busy();
    }
    return LR1110_HAL_STATUS_ERROR;
}

lr1110_hal_status_t lr1110_hal_write_read( const uint8_t* command, uint8_t* data,
                                           const uint16_t data_length )
{
    if( lr1110_hal_wakeup( ) == LR1110_HAL_STATUS_OK )
    {
        digitalWrite(NSS, 0);

        for( uint16_t i = 0; i < data_length; i++ )
        {
            data[i] = SPI.transfer(command[i]);
        }

        digitalWrite(NSS, 1);

        // 0x011B - LR1110_SYSTEM_SET_SLEEP_OC
        if( ( ( command[0] << 8 ) | command[1] ) != 0x011B )
        {
            return lr1110_hal_wait_on_busy( );
        }
        else
        {
            return LR1110_HAL_STATUS_OK;
        }
    }
    return LR1110_HAL_STATUS_ERROR;
}

void lr1110_hal_reset( )
{
    digitalWrite(RESET, 0);
    delay( 1 );
    digitalWrite(RESET, 1);
}

lr1110_hal_status_t lr1110_hal_wakeup( )
{
        // Wakeup radio
        digitalWrite(NSS, 0);
        digitalWrite(NSS, 1);
return lr1110_hal_wait_on_busy();
 }

 static lr1110_hal_status_t lr1110_hal_wait_on_busy( )
{
    while( digitalRead (busyPin)== 1);
    {
    }
    return LR1110_HAL_STATUS_OK;
}
