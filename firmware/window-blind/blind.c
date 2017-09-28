/* blind.c - firmware source for window-blind driver.
 *
 * Copyright © 2016 Sergey Portnov, Eugeny Kaner
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see http://www.gnu.org/licenses/ .
 *
 * Authors: Sergey Portnov <sergius256@gmail.com>
 * Eugeny Kaner<kanerea@xakep.ru>
 *
 * Key debouncer algorithm: Peter Fleury <pfleury@gmx.ch>
 * http://jump.to/fleury, 
 * based on algorithm of Peter Dannegger <danni@specs.de>
 */
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU 8000000UL         // This is for calibrated delays and UART baudrate

#include "../lib/debouncer.c"
#include <util/delay.h>

/*
  Main program is a finite-state machine. So we need to define some
  variables that describes machine state.
 */

unsigned char FSM_State=0, CurrentPosition=0, DesiredPosition=0;
unsigned long int MovementTimeFull,MovementTimeCurrent;

void main(void) {
  DDRB  = 0x03;  // DDRB =0b00000011;
  DDRD  = 0x06;  // DDRD =0b00000110;
  PORTB = 0xFC;  // PORTB=0b11111100;
  PORTD = 0xF8;  // PORTD=0b11111000;

  TCCR0A |= _BV(WGM01);            // CTC mode, OC0A pin disabled
  OCR0A  |=  (int) F_CPU/1024/DEBOUNCE - 1;  // timer = 5 msec
  TIMSK0 |= _BV(OCIE0A);           // enable Output Compare 1 overflow interrupt
  sei();
  TCCR0B |= _BV(CS00)+_BV(CS02);   // F_CPU/1024, see next line
  
  FSM_State=0;

  // Infinite main loop
  for(;;)
    {
      switch(FSM_State)
	{
	case 0:
	  // Switching motor on in forward direction:
	  PORTB=0x05; // 0101
	  FSM_State=1;
	case 1:
	  // _delay_ms(10);
	  // MovementTimeCurrent++;
	  if(key_press & 0x40) // 0x10 // END1 switch _BV(PD4))) //
	    {
	      key_press=0;
	      FSM_State=2;
	    }
	  break;
	case 2:
	  // Switching motor on in backward direction:
	  PORTB=0x06; // 0110
	  FSM_State=3;
	case 3:
	  // _delay_ms(10);
	  // MovementTimeFull++;
	  if(key_press & 0x08) // 0x20 0b0010 0000)) // END2 switch _BV(PD5))) // 
	    {
	      key_press=0;
	      FSM_State=0;
	    }
	}
    }
}
