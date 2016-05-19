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

#include "../lib/parser.c"
#include "../lib/uart.c"

signed char DoAttention(void);
signed char DoSet(void);
signed char DoGet(void);
signed char DoReset(void);
void ParserSetup(void);

/*
  Main program is a finite-state machine. So we need to define some
  variables that describes machine state.
 */

unsigned char FSM_State=0, CurrentPosition=0, DesiredPosition=0;
unsigned long int MovementTimeFull,MovementTimeCurrent;
unsigned char ReportOkFlag;

void main(void) {
  THandler hndl;

  ParserSetup();
  SetDeviceID();
  InitExceptionText();
  FSM_State=0;
  
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

	  DDRB =0b00000011;
	  DDRD =0b00000110;
	  PORTB=0b11111100;
	  PORTD=0b11111000;

	  // SW1 UP -> PB2
	  // SW2 DOWN -> PD7
	  // SW3 END1 -> PD4
	  // SW3 END2 -> PD5

	  // Switching motor on in forward direction
	  PORTB |= 0x001; // 0b00000001;
	  PORTB &= 0x0FD; // 0b11111101;
	  
	  for(MovementTimeCurrent=0;;MovementTimeCurrent++)
	    {
	      _delay_ms(10);
	      if(!(PIND & 0b00010000)) // END1 switch _BV(PD4))) //
		{
		  _delay_ms(50);
		  MovementTimeCurrent+=5;
		  if(!(PIND & 0b00010000)) // _BV(PD4))) //
		    break;
		}
	    }

	  // Switching motor on in backward direction:
	  PORTB &= 0b11111110;
	  PORTB |= 0b00000010;

	  // We have already moved up for MovementTimeCurrent*10
	  // milliseconds so we can safely move back for this time.
	  for(MovementTimeFull=0;MovementTimeFull<=MovementTimeCurrent;MovementTimeFull++)
	    _delay_ms(10);

	  for(;;MovementTimeFull++)
	    {
	      _delay_ms(10);
	      if(!(PIND & 0b00100000)) // END2 switch _BV(PD5))) // 
		{
		  _delay_ms(50);
		  MovementTimeFull+=5;
		  if(!(PIND & 0b00100000)) // _BV(PD5))) // 
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
	  DesiredPosition=(int)MovementTimeCurrent*100/MovementTimeFull;
	  CurrentPosition=100;

	  transmission_ready_flag=0;
	  ReportOkFlag=0;
	  WeAreBusyFlag=0;
	  sei(); // Enable interrupts globally
	  FSM_State=1; // Wait for event
	  break;
	  /*
	    Here we wait for any event possible - button press or command
	    received via UART.
	  */
	case 1: // Wait for event case
	  if(!(PINB & 0b0100)) // UP switch
	    {
	      _delay_ms(50);
	      if(!(PINB & 0b0100)) // still pressed
		DesiredPosition=0;
	    }
	  if(!(PIND & 0b10000000)) // DOWN switch
	    {
	      _delay_ms(50);
	      if(!(PIND & 0b10000000))
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
		      uart_transmit("ERROR_NO_FUNCTION\n");
		      buffer[0]=0;
		      buffer_cur_len=0;
		    }
		  else
		    { // Here we actually call our handler function
		      if (hndl()) // Called function returned error
			{ 
			  // report error
			  uart_transmit("ERROR\n");
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
	  WeAreBusyFlag=0xFF;
	  PORTB |= 0b00000001;
	  PORTB &= 0b11111101;
	  
	  for(;MovementTimeCurrent>=0;MovementTimeCurrent--)
	    {
	      _delay_ms(10);
	      CurrentPosition=(int)MovementTimeCurrent/MovementTimeFull*100;
	      if(CurrentPosition == DesiredPosition)
		break;
	      if(!(PIND & 0b00010000)) // END1 switch reached
		{
		  _delay_ms(50);
		  if(!(PIND & 0b00010000)) // still short
		    {
		      MovementTimeCurrent=0; // We reached the beginning
		      break;
		    }
		}
	      if((!(PIND & 0b10000000)) || (!(PINB & 0b0100))) // UP or DOWN switch pressed
		{
		  _delay_ms(50);
		  MovementTimeCurrent-=5;
		  if((!(PIND & 0b10000000)) || (!(PINB & 0b0100))) // ...still pressed
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
	  WeAreBusyFlag=0;
	  break;
	  /*
	    The only thing we doing here is moving the blind down. The
	    only way to stop this is pressing any button.
	  */
	case 3: // Moving down
	  WeAreBusyFlag=0xFF;
	  PORTB &= 0b11111110;
	  PORTB |= 0b00000010;
	  
	  for(;MovementTimeCurrent<=MovementTimeFull;MovementTimeCurrent++)
	    {
	      _delay_ms(10);
	      CurrentPosition=(int)MovementTimeCurrent/MovementTimeFull*100;
	      if(CurrentPosition == DesiredPosition)
		break;
	      if(!(PIND & 0b00100000)) // END2 switch reached
		{
		  _delay_ms(50);
		  if(!(PIND & 0b00100000)) // still short
		    {
		      MovementTimeCurrent=MovementTimeFull; // We reached the end
		      break;
		    }
		}
	      if((!(PIND & 0b10000000)) || (!(PINB & 0b0100))) // UP or DOWN switch pressed
		{
		  _delay_ms(50);
		  MovementTimeCurrent+=5;
		  if((!(PIND & 0b10000000)) || (!(PINB & 0b0100))) // ...still pressed
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
	  WeAreBusyFlag=0;
	case 4:
	  uart_transmit("OK\n");
	  buffer[0]=0;
	  buffer_cur_len=0;
	  FSM_State=1;
	}
    }
}

/*
  Device-specific functions that implement main fuinctionality.
 */

// Handler function for AT command

signed char DoAttention(void)
{
  uart_transmit("OK\n");
  return 0;
}

// Handler function for SET command

signed char DoSet(void)
{
  int i,pow;
  char number[4];

  for(i=0;(i<4) && (buffer[i]>='0') && (buffer[i]<='9');i++)
    number[i]=buffer[i];

  if((i--)==0)
    return -1;

  pow=1;
  DesiredPosition=0;
  for(;i>=0;i--)
    {
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

signed char DoGet(void)
{
  char number[4];
  number[0]='0'+CurrentPosition/100;
  number[1]='0'+(CurrentPosition/10)%10;
  number[2]='0'+CurrentPosition%100;
  number[3]='\n';
  uart_transmit("POS");
  uart_transmit(number);
  return 0;
}

// Handler function for RESET command

signed char DoReset(void)
{
  FSM_State=0; // Silently resets the Finite-State machine
  return 0;
}


void ParserSetup(void)
{
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
}
