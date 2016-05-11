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
 * Authors:
 * Sergey Portnov <sergius256@gmail.com>
 * Eugeny Kaner <kanerea@xakep.ru>
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

#define F_CPU 4915200            // This is for calibrated delays and UART baudrate
#define BAUD 9600                // Baudrate itself

#include <util/setbaud.h>
#include <util/delay.h>

#define ADDR 0x01000001
#define NUM_PARSER_FUNCTIONS 4

#include "../lib/functions.c"

// Some macros borrowed here: http://www.ladyada.net/learn/proj1/blinky.html
// These are needed to alter pins connected to the peripherals
#define output_low(port,pin) port &= ~(1<<pin)
#define output_high(port,pin) port |= (1<<pin)
#define set_input(portdir,pin) portdir &= ~(1<<pin)
#define set_output(portdir,pin) portdir |= (1<<pin)

// UART initialization function.
void uart_init(void) {
  set_output(DDRD, PD7);  // PD7 pin is connected to RE/DE pins of MAX485
  output_low(PORTD, PD7); // It should be set to 0 until we really transmitting someting out

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
void uart_transmit(void) {
  unsigned char i=0;
  // Here we need to put PD7 pin to 1 to turn MAX485 into transmitter mode
  output_high(PORTD, PD7);
  // The transmission itself
  do {
    while // Wait for empty transmit buffer
      ( !( UCSR0A & (1<<UDRE0)) );
    UDR0 = txbuffer[i];
    i++;
  } while(txbuffer[i-1]!='\n');
  // Making all things back
  output_low(PORTD, PD7);
}

// Recieving function is an interrupt handler
ISR(USART_RX_vect, ISR_BLOCK)
{
  rxbuffer[rxbuffer_cur_len]=UDR0;
  if(rxbuffer[rxbuffer_cur_len]=='\n')    
    transmission_ready_flag=0xFF;
  rxbuffer_cur_len++;
}

/*
  Main program is a finite-state machine. So we need to define some
  variables that describes machine state.
 */

unsigned char FSM_State=0, CurrentPosition=0, DesiredPosition=0, ProtectiveCounter=0;
int MovementTimeFull,MovementTimeCurrent;
unsigned char ReportOkFlag;

/*
  Here we place some device-specific functions that implement main
  fuinctionality.
 */

// Handler function for AT command

signed char DoAttention(void) {
  short int i;
  for(i=0;i<INIT_LEN;i++)
    txbuffer[i]=device_id[i];
  txbuffer[i]='O';  i++;
  txbuffer[i]='K';  i++;
  txbuffer[i]='\n'; i++;
  txbuffer[i]=0;
  return 0;
}

// Handler function for SET command

signed char DoSet(void) {
  int i,pow;
  char number[4];

  for(i=0;(i<4) && (buffer[i]>='0') && (buffer[i]<='9');i++)
    number[i]=buffer[i];

  if((i--)==0)
    return -1;

  pow=1;
  DesiredPosition=0;
  for(;i>=0;i--) {
    DesiredPosition+=(number[i]-'0')*pow;
    pow=pow*10;
  }

  // Some sanity check first
  if((DesiredPosition<0) || (DesiredPosition>100))
    return -1;
  // After this DesiredPosition will be set to some value. If
  // DesiredPosition != CurrentPosition then finite-state machine will
  // handle this situation.
  ReportOkFlag=0xFF;
  return 0;
}

// Handler function for GET command

signed char DoGet(void) {
  short int i;
  for(i=0;i<INIT_LEN;i++)
    txbuffer[i]=device_id[i];
  txbuffer[i]='P';  i++;
  txbuffer[i]='O';  i++;
  txbuffer[i]='S';  i++;
  txbuffer[i]='0'+CurrentPosition/100; i++;
  txbuffer[i]='0'+(CurrentPosition/10)%10; i++;
  txbuffer[i]='0'+CurrentPosition%100; i++;
  txbuffer[i]='\n'; i++;
  txbuffer[i]=0;
  return 0;
}

// Handler function for RESET command

signed char DoReset(void) {
  FSM_State=0; // Silently resets the Finite-State machine
  return 0;
}

void main(void) {
  short int i;
  THandler hndl;

  ParserFunctions[0].name_len=2;
  ParserFunctions[0].name[0]='A';
  ParserFunctions[0].name[1]='T';
  ParserFunctions[0].name[2]=0;
  ParserFunctions[0].handler=DoAttention;

  ParserFunctions[1].name_len=3;
  ParserFunctions[1].name[0]='S';
  ParserFunctions[1].name[1]='E';
  ParserFunctions[1].name[2]='T';
  ParserFunctions[1].name[3]=0;
  ParserFunctions[1].handler=DoSet;
  
  ParserFunctions[2].name_len=3;
  ParserFunctions[2].name[0]='G';
  ParserFunctions[2].name[1]='E';
  ParserFunctions[2].name[2]='T';
  ParserFunctions[2].name[3]=0;
  ParserFunctions[2].handler=DoGet;

  ParserFunctions[3].name_len=5;
  ParserFunctions[3].name[0]='R';
  ParserFunctions[3].name[1]='E';
  ParserFunctions[3].name[2]='S';
  ParserFunctions[3].name[3]='E';
  ParserFunctions[3].name[4]='T';
  ParserFunctions[3].name[5]=0;
  ParserFunctions[3].handler=DoReset;

  SetDeviceID();

  // Infinite main loop
  for(;;){
    switch(FSM_State){
      /*
	Here we initialize our system. This state is default after
	power on. It is also possible to get here using RESET command.
       */
    case 0: // Initialize system
      uart_init();
      set_output(DDRB, PB0);  // PB0 pin is connected to L298 IN1
      set_output(DDRB, PB1);  // PB1 pin is connected to L298 IN2
      
      DDRD=0b10000011;
      
      /*      set_input(DDRD, PD2); // DOWN button
      set_input(DDRD, PD3); // UP button
      set_input(DDRD, PD4); // END1 contact
      set_input(DDRD, PD5); // END2 contact */
      PORTD |= 0b00111100;  // Switching pull-up resistors on

      output_high(PORTB, PB0);
      output_low(PORTB, PB1); // Switching motor on in forward direction
      

      output_low(PORTB, PB0);
      output_high(PORTB, PB1); // Switching motor on in backward direction


      output_low(PORTB, PB0);
      output_low(PORTB, PB1); // Switching motor off
      
      // now MovementTimeCurrent contains number of 10 ms time cycles
      // from current posisiton to upper and MovementTimeFull contains
      // number of 10 ms time cycles from one end to another.

      FSM_State=1; // Wait for event
      break;
      /*
	Here we wait for any event possible - button press or command
	received via UART.
       */
    case 1: // Wait for event case
      // If no button is pressed, ProtectiveCounter==0.
      if(((PIND & 0b0100) == 0) || ((PIND & 0b1000) == 0))// DOWN or UP button pressed
	ProtectiveCounter++;
      _delay_ms(20);
      if((ProtectiveCounter>2) && ((PIND & 1<<PD2) == 0)) { // DOWN button still pressed
	DesiredPosition=0;
      }
      if((ProtectiveCounter>2) && ((PIND & 1<<PD3) == 0)) { // DOWN button still pressed
	DesiredPosition=100;
      }
      // Now let's check our command buffer
      if(transmission_ready_flag) {
	CopyFromRXtoParser();
	transmission_ready_flag=0;
	// Check if command in buffer is for us
	if(IsTransmissionToOurs()){
	  // Ok, we have a command in buffer. We need to send something 
	  if((hndl=Parser())==NULL) {
	    for(i=0;i<INIT_LEN;i++)
	      txbuffer[i]=device_id[i];
	    txbuffer[i]='E';  i++;
	    txbuffer[i]='R';  i++;
	    txbuffer[i]='R';  i++;
	    txbuffer[i]='O';  i++;
	    txbuffer[i]='R';  i++;
	    txbuffer[i]='_';  i++;
	    txbuffer[i]='N';  i++;
	    txbuffer[i]='O';  i++;
	    txbuffer[i]='_';  i++;
	    txbuffer[i]='F';  i++;
	    txbuffer[i]='U';  i++;
	    txbuffer[i]='N';  i++;
	    txbuffer[i]='C';  i++;
	    txbuffer[i]='T';  i++;
	    txbuffer[i]='I';  i++;
	    txbuffer[i]='O';  i++;
	    txbuffer[i]='N';  i++;
	    txbuffer[i]='\n'; i++;
	    txbuffer[i]=0;

	    uart_transmit();
	    buffer[0]=0;
	    buffer_cur_len=0;
	  }
	  else{
	    if (hndl()) { // Called function returned error
	      // report error
	      for(i=0;i<INIT_LEN;i++)
		txbuffer[i]=device_id[i];
	      txbuffer[i]='E';  i++;
	      txbuffer[i]='R';  i++;
	      txbuffer[i]='R';  i++;
	      txbuffer[i]='O';  i++;
	      txbuffer[i]='R';  i++;
	      txbuffer[i]='\n'; i++;
	      txbuffer[i]=0;
	      uart_transmit();
	      buffer[0]=0;
	      buffer_cur_len=0;
	    }
	}
      }
      
      // Finally if somethig happened then DeiredPosition != CurrentPosition
      if(DesiredPosition<CurrentPosition)
	FSM_State=2;
      if(DesiredPosition>CurrentPosition)
	FSM_State=3;      
      break;
      /*
	The only thing we doing here is moving the blind up. The only
	way to stop this is pressing any button.
       */
    case 2: // Moving up
      output_high(PORTB, PB0);
      output_low(PORTB, PB1); // Switching motor on in forward direction

      for(ProtectiveCounter=0;ProtectiveCounter<2;MovementTimeCurrent--) {
	_delay_ms(10);
	if(((PIND & 1<<PD4) == 0) || ((PIND & 1<<PD2) == 0) || ((PIND & 1<<PD3) == 0))
	  ProtectiveCounter++;
	if(CurrentPosition == DesiredPosition || MovementTimeCurrent <= 0)
	  break;
	CurrentPosition=(int)MovementTimeCurrent/MovementTimeFull*100;
      }

      output_low(PORTB, PB0);
      output_low(PORTB, PB1); // Switching motor off
      DesiredPosition=CurrentPosition;
      ProtectiveCounter=0;
      if(ReportOkFlag)
	FSM_State=4;
	else
	  FSM_State=1;
      break;
      /*
	The only thing we doing here is moving the blind down. The
	only way to stop this is pressing any button.
       */
    case 3: // Moving down
      output_low(PORTB, PB0);
      output_high(PORTB, PB1); // Switching motor on in backward direction

      
      for(ProtectiveCounter=0;ProtectiveCounter<2;MovementTimeCurrent++) {
	_delay_ms(10);
	if(((PIND & 1<<PD5) == 0) || ((PIND & 1<<PD2) == 0) || ((PIND & 1<<PD3) == 0))
	  ProtectiveCounter++;
	if(CurrentPosition == DesiredPosition || MovementTimeCurrent >= MovementTimeFull)
	  break;
	CurrentPosition=(int)MovementTimeCurrent/MovementTimeFull*100;
      }

      output_low(PORTB, PB0);
      output_low(PORTB, PB1); // Switching motor off
      DesiredPosition=CurrentPosition;
      ProtectiveCounter=0;
      if(ReportOkFlag)
	FSM_State=4;
	else
	  FSM_State=1;
      break;
      /*
	Special state that comes when movement was caused by UART
	command. We need to report back that we are finished.
       */
      case 4:
	for(i=0;i<INIT_LEN;i++)
	  txbuffer[i]=device_id[i];
	txbuffer[i]='O';  i++;
	txbuffer[i]='K';  i++;
	txbuffer[i]='\n'; i++;
	txbuffer[i]=0;
	uart_transmit();
	FSM_State=1;
      }
    }
  }
}
