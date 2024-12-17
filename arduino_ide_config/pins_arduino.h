/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

// Workaround for wrong definitions in "iom32u4.h".
// This should be fixed in the AVR toolchain.
#undef UHCON
#undef UHINT
#undef UHIEN
#undef UHADDR
#undef UHFNUM
#undef UHFNUML
#undef UHFNUMH
#undef UHFLEN
#undef UPINRQX
#undef UPINTX
#undef UPNUM
#undef UPRST
#undef UPCONX
#undef UPCFG0X
#undef UPCFG1X
#undef UPSTAX
#undef UPCFG2X
#undef UPIENX
#undef UPDATX
#undef TCCR2A
#undef WGM20
#undef WGM21
#undef COM2B0
#undef COM2B1
#undef COM2A0
#undef COM2A1
#undef TCCR2B
#undef CS20
#undef CS21
#undef CS22
#undef WGM22
#undef FOC2B
#undef FOC2A
#undef TCNT2
#undef TCNT2_0
#undef TCNT2_1
#undef TCNT2_2
#undef TCNT2_3
#undef TCNT2_4
#undef TCNT2_5
#undef TCNT2_6
#undef TCNT2_7
#undef OCR2A
#undef OCR2_0
#undef OCR2_1
#undef OCR2_2
#undef OCR2_3
#undef OCR2_4
#undef OCR2_5
#undef OCR2_6
#undef OCR2_7
#undef OCR2B
#undef OCR2_0
#undef OCR2_1
#undef OCR2_2
#undef OCR2_3
#undef OCR2_4
#undef OCR2_5
#undef OCR2_6
#undef OCR2_7

#define NUM_DIGITAL_PINS  18
#define NUM_ANALOG_INPUTS 0

#define TX_RX_LED_INIT	DDRC |= (1<<6), DDRC |= (1<<7)
#define TXLED0			PORTC |= (1<<7)
#define TXLED1			PORTC &= ~(1<<7)
#define RXLED0			PORTC |= (1<<6)
#define RXLED1			PORTC &= ~(1<<6)

#define PIN_WIRE_SDA         (8)
#define PIN_WIRE_SCL         (9)

// port and data direction registers for byte-wise writing PMOD headers
#define DDR_JA  DDRB
#define PORT_JA PORTB
#define DDR_JB  DDRD
#define PORT_JB PORTD

// macros for each pin
#define JA0 0
#define JA1 1
#define JA2 2
#define JA3 3
#define JA4 4
#define JA5 5
#define JA6 6
#define JA7 7
#define JB0 8
#define JB1 9
#define JB2 10
#define JB3 11
#define JB4 12
#define JB5 13
#define JB6 14
#define JB7 15


static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

// #define LED_BUILTIN 12
#define LED_BUILTIN_RX 16
#define LED_BUILTIN_TX 17

// Map SPI port to 'new' pins D14..D17
#define PIN_SPI_SS    (0)
#define PIN_SPI_MOSI  (2)
#define PIN_SPI_MISO  (3)
#define PIN_SPI_SCK   (1)

static const uint8_t SS   = PIN_SPI_SS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

#define digitalPinToPCICR(p)    ( ((p) >= 0 && (p) <= 7) ? (&PCICR) : ((uint8_t *)0))
#define digitalPinToPCICRbit(p) 0
#define digitalPinToPCMSK(p)    (((p) >= 0 && (p) <= 7) ? (&PCMSK0) : ((uint8_t *)0))
#define digitalPinToPCMSKbit(p) (p)

//	__AVR_ATmega32U4__ has an unusual mapping of pins to channels
extern const uint8_t PROGMEM analog_pin_to_channel_PGM[];
#define analogPinToChannel(P)  ( pgm_read_byte( analog_pin_to_channel_PGM + (P) ) )

#define digitalPinHasPWM(p)         ((p) == 8 || (p) == 15 || (p) == 5 || (p) == 6 || (p) == 7)

#define digitalPinToInterrupt(p) ((p) == 10 ? 2 : ((p) == 11 ? 3 : ((p) == 9 ? 1 : ((p) == 8 ? 0 : NOT_AN_INTERRUPT))))

#ifdef ARDUINO_MAIN

// On the Arduino board, digital pins are also used
// for the analog output (software PWM).  Analog input
// pins are a separate set.

// ATMEL ATMEGA32U4 / ARDUINO LEONARDO
//
// D0				PD2					RXD1/INT2
// D1				PD3					TXD1/INT3
// D2				PD1		SDA			SDA/INT1
// D3#				PD0		PWM8/SCL	OC0B/SCL/INT0
// D4		A6		PD4					ADC8
// D5#				PC6		???			OC3A/#OC4A
// D6#		A7		PD7		FastPWM		#OC4D/ADC10
// D7				PE6					INT6/AIN0
//
// D8		A8		PB4					ADC11/PCINT4
// D9#		A9		PB5		PWM16		OC1A/#OC4B/ADC12/PCINT5
// D10#		A10		PB6		PWM16		OC1B/0c4B/ADC13/PCINT6
// D11#				PB7		PWM8/16		0C0A/OC1C/#RTS/PCINT7
// D12		A11		PD6					T1/#OC4D/ADC9
// D13#				PC7		PWM10		CLK0/OC4A
//
// A0		D18		PF7					ADC7
// A1		D19		PF6					ADC6
// A2		D20 	PF5					ADC5
// A3		D21 	PF4					ADC4
// A4		D22		PF1					ADC1
// A5		D23 	PF0					ADC0
//
// New pins D14..D17 to map SPI port to digital pins
//
// MISO		D14		PB3					MISO,PCINT3
// SCK		D15		PB1					SCK,PCINT1
// MOSI		D16		PB2					MOSI,PCINT2
// SS		D17		PB0					RXLED,SS/PCINT0
//
// TXLED	D30		PD5					XCK1
// RXLED	D17	    PB0
// HWB				PE2					HWB

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
	(uint16_t) &DDRE,
	(uint16_t) &DDRF,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
	(uint16_t) &PORTE,
	(uint16_t) &PORTF,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
	(uint16_t) &PINE,
	(uint16_t) &PINF,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {

	// PMOD JA - port B
	PB, // D0  - JA0 - PB0
	PB,	// D1  - JA1 - PB1
	PB, // D2  - JA2 - P2B
	PB,	// D3  - JA3 - PB3
	PB,	// D4  - JA4 - PB4
	PB, // D5  - JA5 - PB5
	PB, // D6  - JA6 - PB6
	PB, // D7  - JA7 - PB7
	
	// PMOD JB - port D
	PD, // D8  - JB0 - PD0
	PD,	// D9  - JB1 - PD1
	PD, // D10 - JB2 - PD2
	PD,	// D11 - JB3 - PD3
	PD, // D12 - JB4 - PD4
	PD, // D13 - JB5 - PD5
	PD,	// D14 - JB6 - PD6
	PD,	// D15 - JB7 - PD7

	PC,
	PC,
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	
	// PMOD JA - port B
	_BV(0), // D0  - JA0 - PB0
	_BV(1), // D1  - JA1 - P1B
	_BV(2), // D2  - JA2 - P2B
	_BV(3), // D3  - JA3 - PB3
	_BV(4), // D4  - JA4 - PB4
	_BV(5), // D5  - JA5 - PB5
	_BV(6), // D6  - JA6 - PB6
	_BV(7), // D7  - JA7 - PB7

	// PMOD JB - port D
	_BV(0), // D8  - JB0 - PD0
	_BV(1), // D9  - JB1 - P1D
	_BV(2), // D10 - JB2 - P2D
	_BV(3), // D11 - JB3 - PD3
	_BV(4), // D12 - JB4 - PD4
	_BV(5), // D13 - JB5 - PD5
	_BV(6), // D14 - JB6 - PD6
	_BV(7), // D15 - JB7 - PD7

	_BV(6), // D16 - RXLED - PC6
	_BV(7), // D17 - TXLED - PC7
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
	NOT_ON_TIMER,	
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,	
	TIMER1A,		/* JA5 */
	TIMER1B,		/* JA6 */
	TIMER0A,		/* JB7 */
	
	TIMER0B,		/* JB0 */
	NOT_ON_TIMER,
	NOT_ON_TIMER,	
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,	
	NOT_ON_TIMER,	
	TIMER4D,		/* JB7 */

	NOT_ON_TIMER,
	NOT_ON_TIMER,
};

const uint8_t PROGMEM analog_pin_to_channel_PGM[] = {};

#endif /* ARDUINO_MAIN */

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
#define SERIAL_PORT_MONITOR        Serial
#define SERIAL_PORT_USBVIRTUAL     Serial
#define SERIAL_PORT_HARDWARE       Serial1
#define SERIAL_PORT_HARDWARE_OPEN  Serial1

// Alias SerialUSB to Serial
#define SerialUSB SERIAL_PORT_USBVIRTUAL

#endif /* Pins_Arduino_h */
