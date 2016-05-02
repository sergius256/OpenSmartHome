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

#define ADDR 0x01000001
#define NUM_PARSER_FUNCTIONS 2

#include "../lib/functions.c"

// Sample function that handles command "SET"
signed char DoSet(void)
{
  printf("Setting %s\n",rcvbuffer);
  return 0;
}

signed char DoAttention(void)
{
  printf("OK\n");
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
  ParserFunctions[0].name_len=3;
  sprintf(ParserFunctions[1].name,"SET");
  ParserFunctions[1].handler=DoSet;

  // setting device_id[] global variable for IsTransmissionToOurs() function
  SetDeviceID();

  // putting some test text into rcvbuffer[] global variable
  sprintf(rcvbuffer,"dev01000001at");

  // debug info
  printf("Command in recieved buffer is %s\nOur device_id is %s\n",rcvbuffer,device_id);
  if(IsTransmissionToOurs()){
    printf("Transmission is for us, command in buffer is %s\n",rcvbuffer);
    if((hndl=Parser())==NULL) {
      // here we should call exception handler that will report error to server
      printf("Parser returned error.\n");
    }
    else
      // just calling proper function and checking if everything is ok
      if (hndl()) {
	// report error
	printf("Function returned error.\n");
      }
      else
	{
	  printf("Reporting success to server.\n");
	}
      
  }
  else
    printf("Transmission is not for us, resulting buffer is %s\n",rcvbuffer);
}
/**/
