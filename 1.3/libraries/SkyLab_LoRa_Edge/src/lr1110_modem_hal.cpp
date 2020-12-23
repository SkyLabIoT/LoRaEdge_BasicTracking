/*The hardware abstraction layer profided by Semtech modifed to work on Arduino platform.*/


#include <SPI.h>
#include <stdlib.h>

#include "lr1110_modem_hal.h"
#include "lr1110_modem_system.h"
#include "lr1110_modem_lorawan.h"
#include "skylabLR.h"

#define LR1110_MODEM_RESET_TIMEOUT 3000

static bool lr1110_modem_reset_timeout = false;

lr1110_modem_event_fields_t  event_fields1;


static lr1110_modem_hal_status_t lr1110_modem_hal_wait_on_unbusy(uint32_t timeout_ms);
static lr1110_modem_hal_status_t lr1110_modem_hal_wait_on_busy(uint32_t timeout_ms);

lr1110_modem_hal_status_t lr1110_modem_hal_write(  const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length )
{
    //Serial.println();
    //Serial.println("............HAL WRITE............");
    if( lr1110_modem_hal_wakeup(  ) == LR1110_MODEM_HAL_STATUS_OK )
    {
        uint8_t                   crc          = 0;
        uint8_t                   crc_received = 0;
        lr1110_modem_hal_status_t status;

        digitalWrite(NSS, 0);
        //Serial.println("Command");
        for( uint16_t i = 0; i < command_length; i++ )
        {
            //Serial.print(command[i]);
            SPI.transfer(command[i]);
        }
        //Serial.println();
        //Serial.println("Data");
        for( uint16_t i = 0; i < data_length; i++ )
        {
            //Serial.print(data[i]);
            SPI.transfer(data[i]);
        }
        crc = lr1110_modem_compute_crc( 0xFF, command, command_length );
        crc = lr1110_modem_compute_crc( crc, data, data_length );
        SPI.transfer(crc);
        //Serial.println("Crc");
        //Serial.println(crc);
        digitalWrite(NSS, 1);
        if( lr1110_modem_hal_wait_on_busy( 1000 ) != LR1110_MODEM_HAL_STATUS_OK )
        {
            return LR1110_MODEM_HAL_STATUS_BUSY_TIMEOUT;
        }

        digitalWrite(NSS, 0);
        status = (lr1110_modem_hal_status_t)SPI.transfer(0);
        //SPI.transfer(0x00);
        crc_received  = SPI.transfer(0);
        crc = lr1110_modem_compute_crc( 0xFF, ( uint8_t* ) &status, 1 );
        //Serial.println("Crc");
        //Serial.println(crc);
        digitalWrite(NSS, 1);
        if( crc != crc_received )
        {
          //Serial.println("No Crc match");
            // change the response code
            status = LR1110_MODEM_HAL_STATUS_BAD_FRAME;
        }
        
        if( ( ( ( command[0] << 8 ) | command[1] ) != 0x0602 ) && ( ( ( command[0] << 8 ) | command[1] ) != 0x0118 ) )
        {
          if( lr1110_modem_hal_wait_on_unbusy( 1000 ) != LR1110_MODEM_HAL_STATUS_OK )
            {
                return LR1110_MODEM_HAL_STATUS_BUSY_TIMEOUT;
            }
          }
        //Serial.println("stat return");
        //Serial.println();
        return status;
    }
    //Serial.println("busy timeout");
    return LR1110_MODEM_HAL_STATUS_BUSY_TIMEOUT;
}

lr1110_modem_hal_status_t lr1110_modem_hal_read( const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length )
{
    //Serial.println();
    //Serial.println("............HAL READ............");
    if( lr1110_modem_hal_wakeup( ) == LR1110_MODEM_HAL_STATUS_OK )
    {
        uint8_t                   crc          = 0;
        uint8_t                   crc_received = 0;
        lr1110_modem_hal_status_t status;

        digitalWrite(NSS, 0);
        //Serial.println("Command");
        for( uint16_t i = 0; i < command_length; i++ )
          {
            //Serial.print(command[i]);
            SPI.transfer(command[i]);
          }
        //Serial.println();
        crc = lr1110_modem_compute_crc( 0xFF, command, command_length );
        //Serial.println("Crc");
        //Serial.println(crc);
        SPI.transfer(crc);

        digitalWrite(NSS, 1);

        //Serial.println("Wait");

        if( lr1110_modem_hal_wait_on_busy( 1000 ) != LR1110_MODEM_HAL_STATUS_OK )
        {
            return LR1110_MODEM_HAL_STATUS_BUSY_TIMEOUT;
        }

        // Send dummy byte
        digitalWrite(NSS, 0);
        status = (lr1110_modem_hal_status_t) SPI.transfer(0x00);
        //SPI.transfer(0x00);
        //Serial.println(status);

        if( status == LR1110_MODEM_HAL_STATUS_OK )
        {
          //Serial.println("Data");
          for( uint16_t i = 0; i < data_length; i++ )
          {
            data[i] = SPI.transfer(0x00);
            //Serial.print(data[i]);
          }
          //Serial.println();
        }
        crc_received = SPI.transfer(0x00);

        digitalWrite(NSS, 1);
       // Serial.println("Crc");
        crc = lr1110_modem_compute_crc( 0xFF, ( uint8_t* ) &status, 1 );
        //Serial.println(crc);
        if( status == LR1110_MODEM_HAL_STATUS_OK )
        {
          //Serial.println("compute Crc");
          crc = lr1110_modem_compute_crc( crc, data, data_length );
          //Serial.println(crc);
        }
        
        if( crc != crc_received )
        {
          //Serial.println("No Crc match");
            // change the response code
            status = LR1110_MODEM_HAL_STATUS_BAD_FRAME;
            //Serial.println(digitalRead(busyPin));
        }
        if( lr1110_modem_hal_wait_on_unbusy( 1000 ) != LR1110_MODEM_HAL_STATUS_OK )
        {
          //Serial.println("Timeout");
          return LR1110_MODEM_HAL_STATUS_BUSY_TIMEOUT;
        }
       //Serial.println("stat return");
       //Serial.println();
       return status;
    }
    //Serial.println("busy timeout");
    return LR1110_MODEM_HAL_STATUS_BUSY_TIMEOUT;
}


lr1110_modem_hal_status_t  lr1110_modem_hal_reset( )
{

    digitalWrite(RESET, 0);
    delay( 1 );
    digitalWrite(RESET, 1);
    delay(100);
    lr1110_modem_get_event( &event_fields1 );
    while (digitalRead (eventPin) == 1){
      lr1110_modem_get_event( &event_fields1 );
      //Serial.println("2");
      }
}

 static lr1110_modem_hal_status_t lr1110_modem_hal_wait_on_busy( uint32_t timeout_ms )
{
    uint32_t start = millis();
    uint32_t current = 0;
    while(digitalRead (busyPin)== 0)
    {
       
        current = millis();
        if( ( int32_t )( current - start ) > ( int32_t ) timeout_ms )
        {
            return LR1110_MODEM_HAL_STATUS_ERROR;
        }
    }
    return LR1110_MODEM_HAL_STATUS_OK;
}

 static lr1110_modem_hal_status_t lr1110_modem_hal_wait_on_unbusy( uint32_t timeout_ms )
{
    uint32_t start = millis();
    uint32_t current = 0;
    while(digitalRead (busyPin)== 1)
    {
       
        current = millis();
        if( ( int32_t )( current - start ) > ( int32_t ) timeout_ms )
        {
            return LR1110_MODEM_HAL_STATUS_ERROR;
        }
    }
    return LR1110_MODEM_HAL_STATUS_OK;
}
void lr1110_modem_hal_enter_dfu()
{
  pinMode (busyPin, OUTPUT);
    // Force dio0 to 0
   digitalWrite(busyPin, 0);

    // reset the chip
    digitalWrite(RESET, 0);
    delay( 1 );
    digitalWrite(RESET, 1);

    // wait 250ms
    delay( 250 );
    pinMode (busyPin, INPUT);
}

static lr1110_modem_hal_status_t lr1110_hal_wait_on_busy1( uint32_t timeout_ms )
{
#if 0
    while (digitalRead (busyPin)== 1)
    {
        ;
    }
#else
    uint32_t start = millis( );
    while(digitalRead (busyPin)== 1)
    {
        if( ( int32_t )( millis( ) - start ) > ( int32_t ) timeout_ms )
        {
            return LR1110_MODEM_HAL_STATUS_ERROR;
        }
    }
#endif
    return LR1110_MODEM_HAL_STATUS_OK;
}

lr1110_modem_hal_status_t lr1110_modem_hal_wakeup( )
{
   if( lr1110_modem_hal_wait_on_busy( 1000 ) == LR1110_MODEM_HAL_STATUS_OK )
    {
        // Wakeup radio
        digitalWrite(NSS, 0);
        SPI.transfer(0x00);
        digitalWrite(NSS, 1);
    }
return lr1110_modem_hal_wait_on_unbusy(1000 );
 }
