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

#define ADDR 0x01000001
#define INIT_LEN 11
#define BUFFER_LEN 256

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

  BUFFER_LEN constant defines the maximum length of string recieved or
  transmitted by slave.
*/

char buffer[BUFFER_LEN], device_id[INIT_LEN+1];

void SetDeviceID(void) {
  /*
    This function should be called before all others to initialize
    device_id pointer.
   */
  sprintf(device_id,"DEV%08x",ADDR);
}

int IsTransmissionToOurs(void) {
  /*
    This function is needed to check if the last command addressing
    our device or not. If yes, device identifier is cleared out from
    buffer to simplify further parsing. If no, buffer gets cleared
    out.
   */
  int i;
  
  if(strncasecmp(buffer,device_id,INIT_LEN)) {
    buffer[0]=0;
    return 0;
  }

  for(i=0;i<strnlen(buffer,BUFFER_LEN)-INIT_LEN;i++)
    buffer[i]=buffer[i+INIT_LEN];
  buffer[i]=0;
  return -1;
}

/*
 main() is for test purposes only here.
 */
/*
int main() {
  SetDeviceID();

  sprintf(buffer,"dev01000001AT");
  printf("buffer=%s\ndevice_id=%s\n",buffer,device_id);
  if(IsTransmissionToOurs()){
    printf("Yes, %s\n",buffer);
  }
  else
    printf("No, %s\n",buffer);
}
*/
