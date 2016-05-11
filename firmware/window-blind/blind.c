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

// UART initialization function.
void uart_init(void) {
  DDRD  |= 0b10000000; // PD7 pin is connected to RE/DE pins of MAX485
  PORTD &= 0b01111111; // It should be set to 0 until we really transmitting someting out

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
  PORTD |= 0b10000000;
  // The transmission itself
  do {
    while // Wait for empty transmit buffer
      ( !( UCSR0A & (1<<UDRE0)) );
    UDR0 = txbuffer[i];
    i++;
  } while(txbuffer[i-1]!='\n');
  // Making all things back
  PORTD &= 0b01111111;
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

unsigned char FSM_State=0, CurrentPosition=0, DesiredPosition=0;
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
  for(;;)
    {
      switch(FSM_State)
	{
	  /*
	    Here we initialize our system. This state is default after
	    power on. It is also possible to get here using RESET command.
	  */
	case 0: // Initialize system
	  uart_init();

	  DDRB=0b00000011;
	  DDRD=0b00000110;

	  // SW1 UP -> PB2
	  // SW2 DOWN -> PD7
	  // SW3 END1 -> PD4
	  // SW3 END2 -> PD5

	  // Switching motor on in forward direction
	  PORTB |= 0b00000001;
	  PORTB &= 0b11111101;
	  
	  for(MovementTimeCurrent=0;;MovementTimeCurrent++)
	    {
	      _delay_ms(10);
	      if((PIND & 0b00010000) == 0) // END1 switch
		{
		  _delay_ms(50);
		  MovementTimeCurrent+=5;
		  if((PIND & 0b00010000) == 0)
		    break;
		}
	    }

	  // Switching motor on in backward direction:
	  PORTB &= 0b11111110;
	  PORTB |= 0b00000010;

	  for(MovementTimeFull=0;;MovementTimeFull++)
	    {
	      _delay_ms(10);
	      if((PIND & 0b00100000) == 0) // END2 switch
		{
		  _delay_ms(50);
		  MovementTimeFull+=5;
		  if((PIND & 0b00100000) == 0)
		    break;
		}
	    }

	  // Switching motor off
	  PORTB &= 0b11111100;
      
	  // now MovementTimeCurrent contains number of 10 ms time
	  // cycles from starting posisiton to upper and
	  // MovementTimeFull contains number of 10 ms time cycles
	  // from one end to another. Here we set DesiredPosition
	  // because actual current position is "fully closed".
	  DesiredPosition=(int)MovementTimeCurrent/MovementTimeFull*100;
	  CurrentPosition=100;

	  transmission_ready_flag=0;
	  ReportOkFlag=0;
	  sei(); // Enable interrupts globally
	  FSM_State=1; // Wait for event
	  break;
	  /*
	    Here we wait for any event possible - button press or command
	    received via UART.
	  */
	case 1: // Wait for event case
	  if((PINB & 0b0100) == 0) // UP switch
	    {
	      _delay_ms(50);
	      if((PINB & 0b0100) == 0) // still pressed
		DesiredPosition=0;
	    }
	  if((PIND & 0b10000000) == 0) // DOWN switch
	    {
	      _delay_ms(50);
	      if((PIND & 0b10000000) == 0)
		  DesiredPosition=100;
	    }

	  // Now let's check our command buffer
      	  if(transmission_ready_flag)
	    {
	      CopyFromRXtoParser();
	      transmission_ready_flag=0;
	      // Check if command in buffer is for us
	      if(IsTransmissionToOurs())
		{
		  // Ok, we have a command in buffer. We need to send something 
		  if((hndl=Parser())==NULL) // If no proper function avilable...
		    {
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
		  else
		    { // Here we actually call our handler function
		      if (hndl()) // Called function returned error
			{ 
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
	    }

	  // Finally if somethig happened then DeiredPosition != CurrentPosition
	  if(DesiredPosition<CurrentPosition) // This will also happen
					      // after initial
					      // movements
	    FSM_State=2;
	  if(DesiredPosition>CurrentPosition)
	    FSM_State=3;      
	  break;
	  /*
	    The only thing we doing here is moving the blind up. The only
	    way to stop this is pressing any button.
	  */
	case 2: // Moving up
	  PORTB |= 0b00000001;
	  PORTB &= 0b11111101;
	  
	  for(;MovementTimeCurrent>=0;MovementTimeCurrent--)
	    {
	      _delay_ms(10);
	      CurrentPosition=(int)MovementTimeCurrent/MovementTimeFull*100;
	      if(CurrentPosition == DesiredPosition)
		break;
	      if((PIND & 0b00010000) == 0) // END1 switch reached
		{
		  _delay_ms(50);
		  if((PIND & 0b00010000) == 0) // still short
		    {
		      MovementTimeCurrent=0; // We reached the beginning
		      break;
		    }
		}
	      if(((PIND & 0b10000000) == 0) || ((PINB & 0b0100) == 0)) // UP or DOWN switch pressed
		{
		  _delay_ms(50);
		  MovementTimeCurrent-=5;
		  if(((PIND & 0b10000000) == 0) || ((PINB & 0b0100) == 0)) // ...still pressed
		    break;
		}
	    }

	  PORTB &= 0b11111100; // Switching motor off

	  if(MovementTimeCurrent < 0) // This can happen when we press
				      // UP or DOWN switch 50 ms to
				      // upper position
	    MovementTimeCurrent=0;
	  // CurrentPosition needs to be corrected when we break'ed by
	  // first or second "if" statement:
	  CurrentPosition=(int)MovementTimeCurrent/MovementTimeFull*100; 
	  
	  DesiredPosition=CurrentPosition;
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
	  PORTB &= 0b11111110;
	  PORTB |= 0b00000010;
	  
	  for(;MovementTimeCurrent<=MovementTimeFull;MovementTimeCurrent++)
	    {
	      _delay_ms(10);
	      CurrentPosition=(int)MovementTimeCurrent/MovementTimeFull*100;
	      if(CurrentPosition == DesiredPosition)
		break;
	      if((PIND & 0b00100000) == 0) // END2 switch reached
		{
		  _delay_ms(50);
		  if((PIND & 0b00100000) == 0) // still short
		    {
		      MovementTimeCurrent=MovementTimeFull; // We reached the end
		      break;
		    }
		}
	      if(((PIND & 0b10000000) == 0) || ((PINB & 0b0100) == 0)) // UP or DOWN switch pressed
		{
		  _delay_ms(50);
		  MovementTimeCurrent+=5;
		  if(((PIND & 0b10000000) == 0) || ((PINB & 0b0100) == 0)) // ...still pressed
		    break;
		}
	    }

	  PORTB &= 0b11111100; // Switching motor off

	  if(MovementTimeCurrent > MovementTimeFull)
	    MovementTimeCurrent = MovementTimeFull;
	  CurrentPosition=(int)MovementTimeCurrent/MovementTimeFull*100; 
	  
	  DesiredPosition=CurrentPosition;
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
