/*
  Copyright (c) 2014-2016 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#pragma once

//#ifndef _VARIANT_SKYLAB_BOARD_
//#define _VARIANT_SKYLAB_BOARD_

// The definitions here needs a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC		(32768ul)

/** Master clock frequency */
#define VARIANT_MCK			  (48000000ul)

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include <WVariant.h>

#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"
#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define PINS_COUNT           (32u)
#define NUM_DIGITAL_PINS     (19u)
#define NUM_ANALOG_INPUTS    (2u)
#define NUM_ANALOG_OUTPUTS   (1u)
//#define analogInputToDigitalPin(p)  ((p < 6u) ? (p) + 24u : -1)

#define digitalPinToPort(P)        ( &(PORT->Group[g_APinDescription[P].ulPort]) )
#define digitalPinToBitMask(P)     ( 1 << g_APinDescription[P].ulPin )
//#define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &(port->OUT.reg) )
#define portInputRegister(port)    ( &(port->IN.reg) )
#define portModeRegister(port)     ( &(port->DIR.reg) )
#define digitalPinHasPWM(P)        ( g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER )

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAMD
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)

// LEDs
#define LEDG1 (18ul)
#define LEDB1 (19ul)
#define LEDR1 (17ul)
static const uint8_t LEDG = LEDG1;
static const uint8_t LEDB = LEDB1;
static const uint8_t LEDR = LEDR1;


/*
 * custom pins
 */
#define PIN_A0               (27ul)
#define PIN_A5               (28ul)
#define PIN_DAC0             (27ul)

#define PA271 				 (6ul)
#define PA281 				 (7ul)
#define PB021 				 (8ul)
#define PB031 				 (9ul)
#define D01  				 (2ul)
#define D31  				 (3ul)
#define D41  				 (4ul)
#define D81 				 (5ul)
#define BUSY1 				 (14ul)
#define EVENT1 				 (13ul)
#define NRESET1 				 (15ul)
#define INT11 				 (10ul)
#define INT21 				 (11ul)
#define INT31 				 (12ul)
#define LNA1  				 (16ul)

static const uint8_t PA27 = PA271 ;
static const uint8_t PA28 = PA281 ;
static const uint8_t PB02 = PB021 ;
static const uint8_t PB03 = PB031 ;
static const uint8_t D0  = D01  ;
static const uint8_t D3  = D31  ;

static const uint8_t D4  = D41  ;
static const uint8_t D8 = D81 ;
static const uint8_t BUSY = BUSY1 ;
static const uint8_t EVENT = EVENT1 ;
static const uint8_t NRESET = NRESET1 ;
static const uint8_t INT1 = INT11 ;
static const uint8_t INT2 = INT21 ;
static const uint8_t INT3 = INT31 ;
static const uint8_t LNA  = LNA1  ;



static const uint8_t A0  = PIN_A0;
static const uint8_t AIN5  = PIN_A5;
static const uint8_t DAC0 = PIN_DAC0;
#define ADC_RESOLUTION		12


/*
 * Serial interfaces
 */
#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"

extern SERCOM sercom0;
extern SERCOM sercom1;
extern SERCOM sercom2;
extern SERCOM sercom3;
extern SERCOM sercom4;
extern SERCOM sercom5;
// Serial1
extern Uart Serial1;
#define PIN_SERIAL1_RX (0ul)
#define PIN_SERIAL1_TX (1ul)
#define PAD_SERIAL1_TX (UART_TX_PAD_2)
#define PAD_SERIAL1_RX (SERCOM_RX_PAD_3)
#endif // __cplusplus

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO         (23u)
#define PIN_SPI_MOSI         (24u)
#define PIN_SPI_SCK          (26u)
#define PERIPH_SPI           sercom1
#define PAD_SPI_TX           SPI_PAD_0_SCK_1
#define PAD_SPI_RX           SERCOM_RX_PAD_3
#define SS          		 (25u)

static const uint8_t NSS	  = SS;	
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (21u)
#define PIN_WIRE_SCL         (22u)
#define PERIPH_WIRE          sercom3
#define WIRE_IT_HANDLER      SERCOM3_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

/*
 * USB
 */
#define PIN_USB_DM          (29ul)
#define PIN_USB_DP          (30ul)

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

/*	=========================
 *	===== SERCOM DEFINITION
 *	=========================
*/


//extern Uart Serial;
extern Uart Serial1;

#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      SerialUSB
#define SERIAL_PORT_MONITOR         SerialUSB
#define SERIAL_PORT_HARDWARE        Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial1

#define Serial                      SerialUSB

//#endif /* _VARIANT_SKYLAB_BOARD_ */

