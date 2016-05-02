/* functions.c - a set of common functions used in OpenSmartHome
 * hardware pieces firmware.
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifndef ADDR
#define ADDR 0x01000001
#endif

#ifndef INIT_LEN
#define INIT_LEN 11
#endif

#ifndef BUFFER_LEN
#define BUFFER_LEN 64
#endif

#ifndef NUM_PARSER_FUNCTIONS
#define NUM_PARSER_FUNCTIONS 1
#endif

#ifndef COMMAND_NAME_LEN
#define COMMAND_NAME_LEN 8
#endif

/*
  Each single device recieves its unique identifier by redefining ADDR
  constant inside main source file. These unique identifiers plays
  role similar to MAC addresses in network controllers.

  Communication inside controllers network is based on queries sent by
  server and answers given by slaves. Server needs to know identifiers
  of all devices connected to it. Server query starts with a plaintext
  containing "DEV" keyword concatenated with slaves identifier,
  i.e. "DEV01000001AT\n" means "Attention to device with ID 01000001.
  The length of such query preamble is always the same, i.e. when
  4-byte ID is used, preamble is 11 bytes of plain text long.

  So INIT_LEN constant points to this preamble length.

  Each message is first accumulated inside rcvbuffer[] array. BUFFER_LEN
  constant defines the maximum length of string recieved or
  transmitted by slave.

  Command parsing logic is divided into 3 steps:

  1. First we need to figure out is transmitted command addressed to
  our device. If not, we flush command buffer and do noting. Otherwise
  we shift buffer contents INIT_LEN symbols left removing
  preamble. All this work is concentrated in IsTransmissionToOurs()
  function. If it return non-zero value, parser function shouldbe
  called to start next step.

  2. Parser() funcition knows about quantity of functions realised by
  our device via NUM_PARSER_FUNCTIONS constant. When called, Parser()
  compares the beginning of the buffer with name of each command. In
  case on success it shifts buffer N bytes left where N is equal to
  command length in symbols, and returns the address of handler
  function. THandler typedef is described for this purpose.

  3. Handler function seeks the parameters it needed inside the
  buffer, does the job and returnt 0 in case of success or -1 in case
  of error.

  If Parser() is not capable to find proper command in the list of
  functions, it returns NULL.

  Parser() looks into ParserFunctions[] array of structures
  (TParserConfig typedef) that contains command names (char name[16])
  not more than 16 characters long, length of that command names (int
  name_len) and address of function that handles each command (int
  (*handler)(void)). Ths array should be initialized prior to all
  other setup procedures inside main() function.

  Stand-alone rcvbuffer_cur_len variable is made to make calculations
  faster. It gets value inside IsTransmissionToOurs() function and
  used inside Parser() after IsTransmissionToOurs() was called. In
  worst case it is equal to 0 that is safe for program logic.

  Attention: #define ADDR and #define NUM_PARSER_FUNCTIONS should be
  placed prior to #include "../lib/functions.c".
*/

typedef signed char (*THandler)(void);

typedef struct {
  unsigned char name_len;
  char name[COMMAND_NAME_LEN];
  THandler handler;
} TParserConfig;

char rcvbuffer[BUFFER_LEN], device_id[INIT_LEN+1];
TParserConfig ParserFunctions[NUM_PARSER_FUNCTIONS];
unsigned char rcvbuffer_cur_len=0;

void SetDeviceID(void) {
  /*
    This function should be called before all others to initialize
    device_id pointer.
   */
  sprintf(device_id,"DEV%08x",ADDR);
}

signed char IsTransmissionToOurs(void) {
  /*
    This function is needed to check if the last command addressing
    our device or not. If yes, device identifier is cleared out from
    buffer to simplify further parsing. If no, buffer gets cleared
    out.
   */
  unsigned char i;
  
  if(strncasecmp(rcvbuffer,device_id,INIT_LEN)) {
    rcvbuffer[0]=0;
    rcvbuffer_cur_len=0;
    return 0;
  }

  rcvbuffer_cur_len=strnlen(rcvbuffer,BUFFER_LEN);
  for(i=0;i<rcvbuffer_cur_len-INIT_LEN;i++)
    rcvbuffer[i]=rcvbuffer[i+INIT_LEN];
  rcvbuffer[i]=0;
  rcvbuffer_cur_len=i;
  return -1;
}

THandler Parser(void) {
  /*
    This function should be called if transmitted text is addressed to
    our device.
  */
  unsigned char i,j;

  for(j=0;j<NUM_PARSER_FUNCTIONS;j++) {
    if(strncasecmp(rcvbuffer,ParserFunctions[j].name,ParserFunctions[j].name_len)==0) {
      for(i=0;i<rcvbuffer_cur_len-ParserFunctions[j].name_len;i++)
	rcvbuffer[i]=rcvbuffer[i+ParserFunctions[j].name_len];
      rcvbuffer[i]=0;
      rcvbuffer_cur_len=i;
      return ParserFunctions[j].handler;
    }
  }
  return NULL;
}
