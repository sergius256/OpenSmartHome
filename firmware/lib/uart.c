/* uart.c - a set of common functions used in OpenSmartHome hardware
 * pieces firmware.
 *
 * Copyright © 2016 Sergey Portnov
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
 * Authors: Sergey Portnov <sergius256@gmail.com>,
 * Eugeny Kaner <kanerea@xakep.ru>
 */

#include "parser.h"

char txbuffer[BUFFER_LEN], BusyExceptionText[INIT_LEN+7];
unsigned char WeAreBusyFlag=0;

// UART initialization function.
void uart_init(void)
{
  DDRD  |= 0b00000100; // PD2 pin is connected to RE/DE pins of MAX485
  PORTD &= 0b11111011; // It should be set to 0 until we really transmitting someting out

  UBRR0H = UBRRH_VALUE;
  UBRR0L = UBRRL_VALUE;

#if USE_2X
  UCSR0A |= _BV(U2X0);
#else
  UCSR0A &= ~(_BV(U2X0));
#endif

  UCSR0A |= _BV(RXCIE0); // Enable interrupt-driven recieve.

  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00) | _BV(UPM01); /* 8-bit data, parity: even */
  UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */
}

// Function that transmits txbuffer through UART
void uart_transmit(char *str)
{
  unsigned char i;

  // Here we need to put PD2 pin to 1 to turn MAX485 into transmitter mode
  PORTD |= 0b00000100;

  // First send device_id string:
  for(i=0;i<INIT_LEN;i++)
    {
      while // Wait for empty transmit buffer
	( !( UCSR0A & (1<<UDRE0)) );
      UDR0 = device_id[i];
    }

  // The transmission itself
  i=0;
  do
    {
      while // Wait for empty transmit buffer
	( !( UCSR0A & (1<<UDRE0)) );
      UDR0 = str[i];
      i++;
    }
  while(str[i-1]!='\n');

  // Making all things back
  PORTD &= 0b11111011;
}

// Functions used in FSM states 2 and 3

void InitExceptionText(void)
{
  int i;
  for(i=0;i<INIT_LEN;i++)
    BusyExceptionText[i]=device_id[i];
  BusyExceptionText[i]='B';  i++;
  BusyExceptionText[i]='U';  i++;
  BusyExceptionText[i]='S';  i++;
  BusyExceptionText[i]='Y';  i++;
  BusyExceptionText[i]='\n';  i++;
  BusyExceptionText[i]=0;  i++;
}

// This interrupr handler is used to transparently handle busy
// exception

ISR(USART_UDRE_vect) 
{
  static uint8_t txIndex=0;

  UDR0 = BusyExceptionText[txIndex]; 
  txIndex += 1;
  
   if(BusyExceptionText[txIndex] == 0) // end of string
     {   
       txIndex = 0;
       UCSR0B &= ~(1<<UDRIE0); // disable UDRE interrupt
       rxbuffer_cur_len=0;
       transmission_ready_flag=0;
     }
}

// Recieving function is an interrupt handler
ISR(USART_RX_vect, ISR_BLOCK)
{
  rxbuffer[rxbuffer_cur_len]=UDR0;
  if(rxbuffer[rxbuffer_cur_len]=='\n')    
    transmission_ready_flag=0xFF;
  rxbuffer_cur_len++;
  if(WeAreBusyFlag)
    {
      UCSR0B |= (1<<UDRIE0); // enable UDRE interrupt to handle busy
			     // exception
      WeAreBusyFlag=0;
    }
}
