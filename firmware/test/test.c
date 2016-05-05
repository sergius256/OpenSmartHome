/* test.c - a set of routines to test functions common to all firmwares
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
 * Authors: Sergey Portnov <sergius256@gmail.com>
 */
#include <ctype.h>

#define ADDR 0x01000001
#define NUM_PARSER_FUNCTIONS 2

#include "../lib/functions.c"

// Sample function that handles command "SET"
signed char DoSet(void)
{
  int i;
  printf("Buffer=%s",buffer);
  sscanf(buffer,"%d",&i);
  printf("Setting %d\n",i);
  return 0;
}

signed char DoAttention(void)
{
  sprintf(txbuffer,"OK\n");
  return 0;
}

/**/
void main() {
  // describing hndl variable that will recieve value returned by parser
  THandler hndl;

  // initializing ParserFunctiuns global array of functions
  ParserFunctions[0].name_len=2;
  sprintf(ParserFunctions[0].name,"AT");
  ParserFunctions[0].handler=DoAttention;

  ParserFunctions[1].name_len=3;
  sprintf(ParserFunctions[1].name,"SET");
  ParserFunctions[1].handler=DoSet;

  // setting device_id[] global variable for IsTransmissionToOurs() function
  SetDeviceID();

  // putting some test text into rcvbuffer[] global variable
  sprintf(buffer,"dev01000001set100\n");

  // debug info
  printf("Command in recieved buffer is %s\nOur device_id is %s\n",buffer,device_id);

  if(IsTransmissionToOurs()){
    // Ok, we have a command in buffer. We need to send something 
    if((hndl=Parser())==NULL) {
      sprintf(txbuffer,"%sERROR_NO_FUNCTION\n",device_id);
      // uart_transmit();
      buffer[0]=0;
      buffer_cur_len=0;
    }
    else{
      if (hndl()) { // Called function returned error
	// report error
	  sprintf(txbuffer,"%sERROR\n",device_id);
	  //	  uart_transmit();
	  buffer[0]=0;
	  buffer_cur_len=0;
      }
    }
  }
  else
    printf("Transmission is not for us, resulting buffer is %s\n",buffer);
  printf("Txbuffer=%s\n",txbuffer);
}
/**/
